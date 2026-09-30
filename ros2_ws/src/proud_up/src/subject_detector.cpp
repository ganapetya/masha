#include "proud_up/subject_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace proud_up {
namespace {

// Turn an OpenCV rectangle into the SubjectDetection the callers use.
//
// head_frac chooses the aim pixel along the box height:
//   0.0 = top edge, 0.5 = middle, 1.0 = bottom.
// For a YOLO person box we use ~0.45 (torso). For a face we use 0.50
// (middle of the face). The cat plug uses 0.50 (middle of the box).
// The contour is the four corners, so a debug overlay can draw the rectangle.
SubjectDetection box_to_detection(const cv::Rect &r, const char *label, double head_frac) {
  SubjectDetection det;
  det.center.u = r.x + r.width * 0.5;
  det.center.v = r.y + r.height * head_frac;
  det.area = static_cast<double>(r.area());
  det.aspect = static_cast<double>(r.width) / static_cast<double>(std::max(1, r.height));
  det.label = label;
  det.contour = {
      {r.x, r.y},
      {r.x + r.width, r.y},
      {r.x + r.width, r.y + r.height},
      {r.x, r.y + r.height},
  };
  det.approx = det.contour;
  return det;
}

cv::Rect largest(const std::vector<cv::Rect> &boxes) {
  return *std::max_element(boxes.begin(), boxes.end(),
                           [](const cv::Rect &a, const cv::Rect &b) { return a.area() < b.area(); });
}

}  // namespace

struct SubjectDetector::SubjectNet {
  cv::dnn::Net net;
};

SubjectDetector::SubjectDetector(const SubjectDetectConfig &cfg) : cfg_(cfg) {
  const auto haar = [&](const char *file) { return cfg_.cascade_dir + "/" + file; };
  const auto lbp = [&](const char *file) { return cfg_.lbp_dir + "/" + file; };
  // LBP is much faster than Haar on this CPU. Haar alt2 is only the fallback
  // if the LBP xml files are missing.
  ok_ = face_.load(lbp("lbpcascade_frontalface.xml")) ||
        face_.load(lbp("lbpcascade_frontalface_improved.xml")) ||
        face_.load(haar("haarcascade_frontalface_alt2.xml"));

  if (!cfg_.person_onnx.empty()) {
    try {
      auto net = std::make_unique<SubjectNet>();
      net->net = cv::dnn::readNetFromONNX(cfg_.person_onnx);
      subject_ = std::move(net);
      subject_ok_ = true;
      if (cfg_.dnn_cuda) {
        subject_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        subject_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        try {
          warmup_subject_net();
          using_cuda_ = true;
        } catch (const cv::Exception &) {
          subject_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
          subject_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
          warmup_subject_net();
          using_cuda_ = false;
        }
      } else {
        subject_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        subject_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        warmup_subject_net();
      }
    } catch (const cv::Exception &) {
      subject_.reset();
      subject_ok_ = false;
      using_cuda_ = false;
    }
  }
}

SubjectDetector::~SubjectDetector() = default;

void SubjectDetector::reset_track() {
  have_last_subject_ = false;
  last_subject_ = cv::Rect();
}

void SubjectDetector::apply_runtime_cfg(const SubjectDetectConfig &cfg) {
  cfg_.image_scale = cfg.image_scale;
  cfg_.scale_factor = cfg.scale_factor;
  cfg_.min_neighbors = cfg.min_neighbors;
  cfg_.min_face = cfg.min_face;
  cfg_.person_conf = cfg.person_conf;
  cfg_.person_iou = cfg.person_iou;
  cfg_.enable_face_refine = cfg.enable_face_refine;
  cfg_.person_head_frac = cfg.person_head_frac;
  cfg_.max_box_frac = cfg.max_box_frac;
  cfg_.max_aspect = cfg.max_aspect;
  cfg_.track_gate_frac = cfg.track_gate_frac;
  cfg_.coco_class = cfg.coco_class;
  cfg_.require_person_shape = cfg.require_person_shape;
  cfg_.enable_face_fallback = cfg.enable_face_fallback;
}

bool SubjectDetector::looks_like_subject(const cv::Rect &r, int width, int height,
                                         const SubjectDetectConfig &cfg) {
  if (!cfg.require_person_shape) {
    if (r.width < 12 || r.height < 12) {
      return false;
    }
    const double frac =
        static_cast<double>(r.area()) / static_cast<double>(std::max(1, width * height));
    return frac <= 0.95;
  }
  if (r.width < 20 || r.height < 32) {
    return false;
  }
  const double aspect = static_cast<double>(r.width) / static_cast<double>(std::max(1, r.height));
  if (aspect > cfg.max_aspect) {
    return false;  // a sofa or a curtain is usually wider than it is tall
  }
  const double frac =
      static_cast<double>(r.area()) / static_cast<double>(std::max(1, width * height));
  if (frac > cfg.max_box_frac || frac < 0.008) {
    return false;
  }
  // A box stuck to the top of the frame, covering less than half the
  // height, is almost always the curtain rail, not a torso.
  if (r.y <= 6 && r.height < static_cast<int>(height * 0.55)) {
    return false;
  }
  return true;
}

void SubjectDetector::warmup_subject_net() {
  if (!subject_) {
    return;
  }
  const int sz = cfg_.person_imgsz > 0 ? cfg_.person_imgsz : 320;
  cv::Mat dummy(sz, sz, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat blob = cv::dnn::blobFromImage(dummy, 1.0 / 255.0, cv::Size(sz, sz), cv::Scalar(),
                                        true, false);
  subject_->net.setInput(blob);
  subject_->net.forward();
}

std::optional<cv::Rect> SubjectDetector::detect_subject_box(const cv::Mat &bgr) {
  if (!subject_ || bgr.empty()) {
    return std::nullopt;
  }
  const int sz = cfg_.person_imgsz > 0 ? cfg_.person_imgsz : 320;
  const float conf_th = static_cast<float>(cfg_.person_conf);
  const float iou_th = static_cast<float>(cfg_.person_iou);
  cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0, cv::Size(sz, sz), cv::Scalar(),
                                        true, false);
  subject_->net.setInput(blob);
  cv::Mat out = subject_->net.forward();
  if (out.empty() || out.dims < 2) {
    return std::nullopt;
  }
  if (!out.isContinuous()) {
    out = out.clone();
  }

  // Ultralytics YOLOv8 ONNX output is [1, 84, N]:
  //   4 box numbers (centre x, centre y, width, height) plus 80 COCO class
  //   scores, all in the stretched 320×320 blob.
  // We reshape to N × 84 so each row is one candidate.
  cv::Mat pred;
  if (out.dims == 3 && out.size[1] == 84) {
    pred = out.reshape(1, out.size[1]);  // 84 × N
    cv::transpose(pred, pred);           // N × 84
  } else if (out.dims == 2 && out.size[1] == 84) {
    pred = out;
  } else if (out.dims == 3 && out.size[2] == 84) {
    pred = out.reshape(1, out.size[1]);  // N × 84
  } else {
    return std::nullopt;
  }

  const double sx = static_cast<double>(bgr.cols) / static_cast<double>(sz);
  const double sy = static_cast<double>(bgr.rows) / static_cast<double>(sz);
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  if (pred.cols < 5) {
    return std::nullopt;
  }
  for (int i = 0; i < pred.rows; ++i) {
    const float *row = pred.ptr<float>(i);
    // row[4 + coco_class] is that class's score. Class 0 is person, 15 is
    // cat. Class 0 may lose the argmax to "chair" and still be a person.
    // Any other class must be the winning score, or a chair with a weak
    // cat score would play call-kitten.
    const int cls = cfg_.coco_class;
    if (cls < 0 || 4 + cls >= pred.cols) {
      continue;
    }
    const float subject_s = row[4 + cls];
    if (subject_s < conf_th) {
      continue;
    }
    if (cls != 0 && !coco_class_is_best(row, pred.cols, cls)) {
      continue;
    }
    const float cx = row[0];
    const float cy = row[1];
    const float bw = row[2];
    const float bh = row[3];
    const int x = static_cast<int>(std::lround((cx - bw * 0.5f) * sx));
    const int y = static_cast<int>(std::lround((cy - bh * 0.5f) * sy));
    const int ww = static_cast<int>(std::lround(bw * sx));
    const int hh = static_cast<int>(std::lround(bh * sy));
    cv::Rect r(x, y, ww, hh);
    r &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (!looks_like_subject(r, bgr.cols, bgr.rows, cfg_)) {
      continue;
    }
    boxes.push_back(r);
    scores.push_back(subject_s);
  }
  if (boxes.empty()) {
    return std::nullopt;
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, conf_th, iou_th, keep);
  if (keep.empty()) {
    return std::nullopt;
  }

  int best_i = keep[0];
  if (have_last_subject_) {
    // Stay on the same body. Picking the largest box each frame walked
    // the camera onto a window curtain and left it stuck there.
    const cv::Point last_c(last_subject_.x + last_subject_.width / 2,
                           last_subject_.y + last_subject_.height / 2);
    const double gate =
        cfg_.track_gate_frac * std::hypot(static_cast<double>(bgr.cols),
                                          static_cast<double>(bgr.rows));
    double best_d = 1e9;
    int gated = -1;
    for (int idx : keep) {
      const cv::Point c(boxes[idx].x + boxes[idx].width / 2,
                        boxes[idx].y + boxes[idx].height / 2);
      const double d = std::hypot(static_cast<double>(c.x - last_c.x),
                                  static_cast<double>(c.y - last_c.y));
      if (d < best_d) {
        best_d = d;
        gated = idx;
      }
    }
    if (gated >= 0 && best_d <= gate) {
      best_i = gated;
    } else {
      // Jumped outside the gate. Drop the lock this frame so the next
      // frame can acquire anywhere. Keeping the old centre hid a darting cat.
      reset_track();
      return std::nullopt;
    }
  } else {
    for (int idx : keep) {
      if (scores[idx] > scores[best_i]) {
        best_i = idx;
      }
    }
  }
  last_subject_ = boxes[best_i];
  have_last_subject_ = true;
  return boxes[best_i];
}

std::optional<SubjectDetection> SubjectDetector::detect_face_in(const cv::Mat &bgr,
                                                               const cv::Rect &roi) {
  if (!ok_ || bgr.empty()) {
    return std::nullopt;
  }
  cv::Rect region = roi;
  if (region.width <= 0 || region.height <= 0) {
    region = cv::Rect(0, 0, bgr.cols, bgr.rows);
  } else {
    // Search the upper 65% of the subject box, with a little padding, for
    // a face looking at the camera.
    region.height = std::max(1, static_cast<int>(region.height * 0.65));
    region.x = std::max(0, region.x - 8);
    region.y = std::max(0, region.y - 8);
    region.width = std::min(bgr.cols - region.x, region.width + 16);
    region.height = std::min(bgr.rows - region.y, region.height + 16);
  }
  cv::Mat crop = bgr(region);
  cv::Mat gray;
  if (crop.channels() == 1) {
    gray = crop;
  } else {
    cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
  }
  const double s = (cfg_.image_scale > 0.2 && cfg_.image_scale < 1.0) ? cfg_.image_scale : 1.0;
  cv::Mat small;
  if (s < 0.999) {
    cv::resize(gray, small, cv::Size(), s, s, cv::INTER_AREA);
  } else {
    small = gray;
  }
  cv::equalizeHist(small, small);

  std::vector<cv::Rect> hits;
  const int min_px = std::max(16, static_cast<int>(cfg_.min_face * s));
  face_.detectMultiScale(small, hits, cfg_.scale_factor, cfg_.min_neighbors,
                         cv::CASCADE_SCALE_IMAGE, cv::Size(min_px, min_px));
  if (hits.empty()) {
    return std::nullopt;
  }
  cv::Rect f = largest(hits);
  f.x = static_cast<int>(f.x / s) + region.x;
  f.y = static_cast<int>(f.y / s) + region.y;
  f.width = static_cast<int>(f.width / s);
  f.height = static_cast<int>(f.height / s);
  f &= cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (f.width < 8 || f.height < 8) {
    return std::nullopt;
  }
  return box_to_detection(f, "face", 0.50);
}

std::optional<SubjectDetection> SubjectDetector::detect(const cv::Mat &bgr) {
  const auto t0 = std::chrono::steady_clock::now();
  auto finish = [&](std::optional<SubjectDetection> out) {
    last_detect_ms_ = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    return out;
  };

  if (bgr.empty() || (!ok_ && !subject_ok_)) {
    return finish(std::nullopt);
  }

  if (subject_ok_) {
    const auto box = detect_subject_box(bgr);
    if (box) {
      if (cfg_.enable_face_refine) {
        if (auto face = detect_face_in(bgr, *box)) {
          return finish(face);
        }
      }
      // Aim inside the box, not at its top edge. A small head_frac
      // (for example 0.12) pointed the camera at the curtain and the
      // lock-on-centre test then treated that as success.
      const double frac =
          (cfg_.person_head_frac > 0.05 && cfg_.person_head_frac < 0.9) ? cfg_.person_head_frac
                                                                       : 0.45;
      const char *label = (cfg_.coco_class == 15) ? "cat" : "person";
      return finish(box_to_detection(*box, label, frac));
    }
  }

  // Close-up facing the camera with no YOLO box (or no ONNX file loaded).
  // Cat plug turns this off: a face is not a cat.
  if (cfg_.enable_face_fallback) {
    if (auto face = detect_face_in(bgr, cv::Rect())) {
      return finish(face);
    }
  }

  return finish(std::nullopt);
}

}  // namespace proud_up
