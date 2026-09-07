#include "proud_up/follow_the_cat.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace proud_up {
namespace {

int odd_or_zero(int k) {
  if (k < 3) {
    return 0;
  }
  return (k % 2 == 0) ? (k + 1) : k;
}

bool touches_image_border(const std::vector<cv::Point> &contour, int width, int height,
                          int margin) {
  const cv::Rect box = cv::boundingRect(contour);
  const int m = std::max(0, margin);
  return box.x <= m || box.y <= m || (box.x + box.width) >= (width - m) ||
         (box.y + box.height) >= (height - m);
}

bool looks_like_portrait_phone(const std::vector<cv::Point> &contour, const DetectionConfig &cfg,
                               double area, double max_area, std::vector<cv::Point> *approx_out,
                               double *aspect_out, DetectionStats *stats) {
  if (area < cfg.min_area || area > max_area) {
    if (stats) {
      ++stats->rejected_area;
    }
    return false;
  }

  const double peri = cv::arcLength(contour, true);
  std::vector<cv::Point> approx;
  cv::approxPolyDP(contour, approx, cfg.poly_eps_frac * peri, true);
  const int n = static_cast<int>(approx.size());
  const bool vertex_ok = n >= cfg.min_vertices && n <= cfg.max_vertices;
  const bool convex = approx.size() >= 3 && cv::isContourConvex(approx);

  const cv::Rect box = cv::boundingRect(contour);
  const double aspect =
      static_cast<double>(box.width) / static_cast<double>(std::max(1, box.height));
  const bool aspect_ok = aspect >= cfg.min_aspect && aspect <= cfg.max_aspect;
  const bool portrait_ok = !cfg.require_portrait || (box.height > box.width);

  const double box_area = static_cast<double>(std::max(1, box.width * box.height));
  const double solidity = area / box_area;
  const bool solid_ok = solidity >= cfg.min_solidity;

  bool ok = aspect_ok && portrait_ok && solid_ok && convex;
  if (cfg.require_quad) {
    ok = ok && vertex_ok;
  }
  if (!ok) {
    if (stats) {
      ++stats->rejected_shape;
    }
    return false;
  }
  if (approx_out) {
    *approx_out = std::move(approx);
  }
  if (aspect_out) {
    *aspect_out = aspect;
  }
  return true;
}

Pixel centroid(const std::vector<cv::Point> &contour) {
  const cv::Moments m = cv::moments(contour);
  Pixel p;
  if (std::abs(m.m00) < 1e-6) {
    const cv::Rect box = cv::boundingRect(contour);
    p.u = box.x + box.width * 0.5;
    p.v = box.y + box.height * 0.5;
    return p;
  }
  // Spatial moments: (m10/m00, m01/m00) is the area centroid in px.
  p.u = m.m10 / m.m00;
  p.v = m.m01 / m.m00;
  return p;
}

}  // namespace

std::optional<CardDetection> pick_black_portrait(const cv::Mat &gray, const DetectionConfig &cfg,
                                                 int threshold, DetectionStats *stats) {
  // Inverted threshold: dark pixels (the phone) become 255, the white wall
  // becomes 0. A white-card detector could never do this split.
  cv::Mat mask;
  cv::threshold(gray, mask, threshold, 255, cv::THRESH_BINARY_INV);

  const int morph = odd_or_zero(cfg.morph_ksize);
  if (morph > 0) {
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph, morph));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (stats) {
    stats->contours += static_cast<int>(contours.size());
  }

  const double frame_area = static_cast<double>(gray.cols) * static_cast<double>(gray.rows);
  double max_area = cfg.max_area;
  if (cfg.max_area_frac > 0.0) {
    max_area = std::min(max_area, cfg.max_area_frac * frame_area);
  }

  std::optional<CardDetection> best;
  for (const auto &contour : contours) {
    if (cfg.reject_border &&
        touches_image_border(contour, gray.cols, gray.rows, cfg.border_margin)) {
      if (stats) {
        ++stats->rejected_border;
      }
      continue;
    }
    const double area = cv::contourArea(contour);
    std::vector<cv::Point> approx;
    double aspect = 0.0;
    if (!looks_like_portrait_phone(contour, cfg, area, max_area, &approx, &aspect, stats)) {
      continue;
    }
    if (best && area <= best->area) {
      continue;
    }
    CardDetection det;
    det.center = centroid(contour);
    det.contour = contour;
    det.approx = std::move(approx);
    det.area = area;
    det.aspect = aspect;
    best = std::move(det);
  }
  return best;
}

std::optional<CardDetection> detect_black_rectangle(const cv::Mat &bgr, const DetectionConfig &cfg,
                                                    DetectionStats *stats) {
  if (bgr.empty()) {
    return std::nullopt;
  }

  cv::Mat gray;
  if (bgr.channels() == 1) {
    gray = bgr;
  } else {
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  }

  const int blur = odd_or_zero(cfg.blur_ksize);
  if (blur > 0) {
    cv::GaussianBlur(gray, gray, cv::Size(blur, blur), 0.0);
  }

  auto det = pick_black_portrait(gray, cfg, cfg.threshold, stats);
  if (det) {
    return det;
  }
  // Aurora 640x400 is dim; a "black" phone can sit around gray 100. Retry looser.
  if (cfg.loose_threshold > cfg.threshold) {
    if (stats) {
      stats->used_loose_pass = true;
    }
    det = pick_black_portrait(gray, cfg, cfg.loose_threshold, stats);
  }
  return det;
}

namespace {

CardDetection box_to_detection(const cv::Rect &r, const char *label, double head_frac) {
  CardDetection det;
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

struct HumanDetector::PersonNet {
  cv::dnn::Net net;
};

HumanDetector::HumanDetector(const HumanDetectConfig &cfg) : cfg_(cfg) {
  const auto haar = [&](const char *file) { return cfg_.cascade_dir + "/" + file; };
  const auto lbp = [&](const char *file) { return cfg_.lbp_dir + "/" + file; };
  // LBP is ~5–10× faster than Haar on this CPU. Haar alt2 is the fallback.
  ok_ = face_.load(lbp("lbpcascade_frontalface.xml")) ||
        face_.load(lbp("lbpcascade_frontalface_improved.xml")) ||
        face_.load(haar("haarcascade_frontalface_alt2.xml"));
  if (cfg_.enable_upper) {
    have_upper_ = upper_.load(haar("haarcascade_upperbody.xml"));
  }
  if (cfg_.enable_full) {
    have_full_ = full_.load(haar("haarcascade_fullbody.xml"));
  }

  if (!cfg_.person_onnx.empty()) {
    try {
      auto net = std::make_unique<PersonNet>();
      net->net = cv::dnn::readNetFromONNX(cfg_.person_onnx);
      person_ = std::move(net);
      person_ok_ = true;
      if (cfg_.dnn_cuda) {
        person_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        person_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        try {
          warmup_person_net();
          using_cuda_ = true;
        } catch (const cv::Exception &) {
          person_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
          person_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
          warmup_person_net();
          using_cuda_ = false;
        }
      } else {
        person_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        person_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        warmup_person_net();
      }
    } catch (const cv::Exception &) {
      person_.reset();
      person_ok_ = false;
      using_cuda_ = false;
    }
  }
}

HumanDetector::~HumanDetector() = default;

void HumanDetector::reset_track() {
  have_last_person_ = false;
  last_person_ = cv::Rect();
}

void HumanDetector::apply_runtime_cfg(const HumanDetectConfig &cfg) {
  cfg_.image_scale = cfg.image_scale;
  cfg_.scale_factor = cfg.scale_factor;
  cfg_.min_neighbors = cfg.min_neighbors;
  cfg_.min_face = cfg.min_face;
  cfg_.min_upper = cfg.min_upper;
  cfg_.min_full = cfg.min_full;
  cfg_.person_conf = cfg.person_conf;
  cfg_.person_iou = cfg.person_iou;
  cfg_.enable_face_refine = cfg.enable_face_refine;
  cfg_.person_head_frac = cfg.person_head_frac;
  cfg_.max_box_frac = cfg.max_box_frac;
  cfg_.max_aspect = cfg.max_aspect;
  cfg_.track_gate_frac = cfg.track_gate_frac;
}

bool HumanDetector::looks_like_person(const cv::Rect &r, int width, int height,
                                      const HumanDetectConfig &cfg) {
  if (r.width < 20 || r.height < 32) {
    return false;
  }
  const double aspect = static_cast<double>(r.width) / static_cast<double>(std::max(1, r.height));
  if (aspect > cfg.max_aspect) {
    return false;  // curtains / sofa are landscape
  }
  const double frac =
      static_cast<double>(r.area()) / static_cast<double>(std::max(1, width * height));
  if (frac > cfg.max_box_frac || frac < 0.008) {
    return false;
  }
  // A box glued to the top of the frame is the curtain rod, not a torso.
  if (r.y <= 6 && r.height < static_cast<int>(height * 0.55)) {
    return false;
  }
  return true;
}

void HumanDetector::warmup_person_net() {
  if (!person_) {
    return;
  }
  const int sz = cfg_.person_imgsz > 0 ? cfg_.person_imgsz : 320;
  cv::Mat dummy(sz, sz, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat blob = cv::dnn::blobFromImage(dummy, 1.0 / 255.0, cv::Size(sz, sz), cv::Scalar(),
                                        true, false);
  person_->net.setInput(blob);
  person_->net.forward();
}

std::optional<cv::Rect> HumanDetector::detect_person_box(const cv::Mat &bgr) {
  if (!person_ || bgr.empty()) {
    return std::nullopt;
  }
  const int sz = cfg_.person_imgsz > 0 ? cfg_.person_imgsz : 320;
  const float conf_th = static_cast<float>(cfg_.person_conf);
  const float iou_th = static_cast<float>(cfg_.person_iou);
  cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0, cv::Size(sz, sz), cv::Scalar(),
                                        true, false);
  person_->net.setInput(blob);
  cv::Mat out = person_->net.forward();
  if (out.empty() || out.dims < 2) {
    return std::nullopt;
  }
  if (!out.isContinuous()) {
    out = out.clone();
  }

  // Ultralytics YOLOv8 ONNX: [1, 84, N] = (cx,cy,w,h) + 80 COCO scores, in
  // the stretched 320×320 blob. Class 0 is person.
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
    // COCO class 0 = person. Do not require it to beat "chair"/"couch"
    // on the same anchor — a seated person often loses that argmax.
    const float person_s = row[4];
    if (person_s < conf_th) {
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
    if (!looks_like_person(r, bgr.cols, bgr.rows, cfg_)) {
      continue;
    }
    boxes.push_back(r);
    scores.push_back(person_s);
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
  if (have_last_person_) {
    // Stay on the same body. Picking the *largest* box walked the arm up
    // onto the window curtain and stuck there.
    const cv::Point last_c(last_person_.x + last_person_.width / 2,
                           last_person_.y + last_person_.height / 2);
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
      return std::nullopt;
    }
  } else {
    for (int idx : keep) {
      if (scores[idx] > scores[best_i]) {
        best_i = idx;
      }
    }
  }
  last_person_ = boxes[best_i];
  have_last_person_ = true;
  return boxes[best_i];
}

std::optional<CardDetection> HumanDetector::detect_face_in(const cv::Mat &bgr,
                                                          const cv::Rect &roi) {
  if (!ok_ || bgr.empty()) {
    return std::nullopt;
  }
  cv::Rect region = roi;
  if (region.width <= 0 || region.height <= 0) {
    region = cv::Rect(0, 0, bgr.cols, bgr.rows);
  } else {
    // Search the upper 65% of the person box, padded, for a frontal face.
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

std::optional<CardDetection> HumanDetector::detect(const cv::Mat &bgr) {
  const auto t0 = std::chrono::steady_clock::now();
  auto finish = [&](std::optional<CardDetection> out) {
    last_detect_ms_ = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    return out;
  };

  if (bgr.empty() || (!ok_ && !person_ok_)) {
    return finish(std::nullopt);
  }

  if (person_ok_) {
    const auto box = detect_person_box(bgr);
    if (box) {
      if (cfg_.enable_face_refine) {
        if (auto face = detect_face_in(bgr, *box)) {
          return finish(face);
        }
      }
      // Aim at the torso, not the top of the box. head_frac=0.12 walked
      // pitch onto the curtain and then locked there.
      const double frac =
          (cfg_.person_head_frac > 0.05 && cfg_.person_head_frac < 0.9) ? cfg_.person_head_frac
                                                                       : 0.45;
      return finish(box_to_detection(*box, "person", frac));
    }
  }

  // Close-up facing the camera with no YOLO box (or no ONNX loaded).
  if (auto face = detect_face_in(bgr, cv::Rect())) {
    return finish(face);
  }

  if (have_upper_ || have_full_) {
    cv::Mat gray;
    if (bgr.channels() == 1) {
      gray = bgr;
    } else {
      cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    }
    const double s = (cfg_.image_scale > 0.2 && cfg_.image_scale < 1.0) ? cfg_.image_scale : 1.0;
    cv::Mat small;
    if (s < 0.999) {
      cv::resize(gray, small, cv::Size(), s, s, cv::INTER_AREA);
    } else {
      small = gray;
    }
    cv::equalizeHist(small, small);
    const auto to_full = [s](const cv::Rect &r) {
      return cv::Rect(static_cast<int>(r.x / s), static_cast<int>(r.y / s),
                      static_cast<int>(r.width / s), static_cast<int>(r.height / s));
    };
    const auto min_on_small = [s](int px) {
      return cv::Size(std::max(16, static_cast<int>(px * s)),
                      std::max(16, static_cast<int>(px * s)));
    };
    std::vector<cv::Rect> hits;
    const cv::Size max_win(small.cols * 3 / 4, small.rows * 3 / 4);
    if (have_upper_) {
      upper_.detectMultiScale(small, hits, std::max(1.25, cfg_.scale_factor), 3,
                              cv::CASCADE_SCALE_IMAGE, min_on_small(cfg_.min_upper), max_win);
      if (!hits.empty()) {
        return finish(box_to_detection(to_full(largest(hits)), "upper_body", 0.22));
      }
    }
    if (have_full_) {
      hits.clear();
      full_.detectMultiScale(small, hits, std::max(1.25, cfg_.scale_factor), 3,
                             cv::CASCADE_SCALE_IMAGE, min_on_small(cfg_.min_full), max_win);
      if (!hits.empty()) {
        return finish(box_to_detection(to_full(largest(hits)), "full_body", 0.18));
      }
    }
  }
  return finish(std::nullopt);
}

CameraIntrinsics fallback_intrinsics() {
  // peripherals/config/camera_info.yaml — USB cam, not a promise about Aurora
  // resolution_mode_index=2. Prefer live camera_info whenever it exists.
  return CameraIntrinsics{};
}

CameraIntrinsics from_k_matrix(int width, int height, const double k[9]) {
  CameraIntrinsics K;
  K.width = width;
  K.height = height;
  // sensor_msgs/CameraInfo.k is row-major 3×3:
  //   k[0]=fx  k[2]=cx
  //   k[4]=fy  k[5]=cy
  K.fx = k[0];
  K.fy = k[4];
  K.cx = k[2];
  K.cy = k[5];
  K.from_camera_info = K.usable();
  return K;
}

Eigen::Vector3d pixel_to_ray(double u, double v, const CameraIntrinsics &K) {
  const double fx = (K.fx > 1e-6) ? K.fx : 1.0;
  const double fy = (K.fy > 1e-6) ? K.fy : 1.0;
  const double X = (u - K.cx) / fx;
  const double Y = (v - K.cy) / fy;
  // Z = 1 is a virtual plane in front of the lens, not a measured depth.
  // Normalizing makes this a unit direction so yaw/pitch do not depend on
  // that arbitrary scale.
  return Eigen::Vector3d(X, Y, 1.0).normalized();
}

GazeAngles ray_to_yaw_pitch(const Eigen::Vector3d &ray) {
  GazeAngles a;
  const double X = ray.x();
  const double Y = ray.y();
  const double Z = ray.z();
  a.yaw = std::atan2(X, Z);
  a.pitch = std::atan2(-Y, std::hypot(X, Z));
  return a;
}

Eigen::Quaterniond gaze_quaternion(double yaw, double pitch) {
  // Eigen multiplies on the left: (q_yaw * q_pitch) * v applies pitch first,
  // then yaw. That matches "tilt the lens, then pan". Day 3 only logs this;
  // hardware gets the two scalar angles, not the quaternion.
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  return q;
}

float clamp_pulse(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

float clamp_delta(float value, float max_abs) {
  const float cap = std::abs(max_abs);
  return std::max(-cap, std::min(cap, value));
}

GazePulses integrate_gaze(const GazePulses &current, const GazeAngles &err,
                          const PulseMapping &map, double gain, double max_step,
                          double deadband_rad) {
  GazePulses out = current;
  double yaw = err.yaw;
  double pitch = err.pitch;
  if (std::abs(yaw) < deadband_rad) {
    yaw = 0.0;
  }
  if (std::abs(pitch) < deadband_rad) {
    pitch = 0.0;
  }
  const double ticks = (map.ticks_per_rad > 1.0) ? map.ticks_per_rad : kTicksPerRadian;
  const double g = (gain > 0.0) ? gain : 0.0;
  const float dpan =
      clamp_delta(static_cast<float>(map.yaw_sign * yaw * ticks * g), static_cast<float>(max_step));
  const float dtilt = clamp_delta(static_cast<float>(map.pitch_sign * pitch * ticks * g),
                                  static_cast<float>(max_step));
  out.id19 = clamp_pulse(current.id19 + dpan, map.pan_min, map.pan_max);
  out.id22 = clamp_pulse(current.id22 + dtilt, map.tilt_min, map.tilt_max);
  return out;
}

GazePulses angles_to_pulses(const GazeAngles &angles, const PulseMapping &map) {
  GazePulses out;
  const double ticks = (map.ticks_per_rad > 1.0) ? map.ticks_per_rad : kTicksPerRadian;
  out.id19 = clamp_pulse(
      static_cast<float>(map.pan_rest + map.yaw_sign * angles.yaw * ticks),
      map.pan_min, map.pan_max);
  out.id22 = clamp_pulse(
      static_cast<float>(map.tilt_rest + map.pitch_sign * angles.pitch * ticks),
      map.tilt_min, map.tilt_max);
  return out;
}

bool near_optical_axis(const GazeAngles &angles, double yaw_tol, double pitch_tol) {
  return std::abs(angles.yaw) <= yaw_tol && std::abs(angles.pitch) <= pitch_tol;
}

void draw_debug_overlay(cv::Mat &bgr, const std::optional<CardDetection> &det,
                        const char *phase, const GazeAngles *angles,
                        const DetectionStats *stats, const char *note) {
  if (bgr.empty()) {
    return;
  }
  const cv::Point frame_center(bgr.cols / 2, bgr.rows / 2);
  cv::drawMarker(bgr, frame_center, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 24, 2);

  if (det) {
    std::vector<std::vector<cv::Point>> one{det->contour};
    cv::drawContours(bgr, one, 0, cv::Scalar(0, 255, 0), 2);
    if (det->approx.size() >= 3) {
      cv::polylines(bgr, det->approx, true, cv::Scalar(0, 255, 255), 2);
    }
    const cv::Point c(static_cast<int>(std::lround(det->center.u)),
                      static_cast<int>(std::lround(det->center.v)));
    cv::circle(bgr, c, 6, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::drawMarker(bgr, c, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 18, 1);
    cv::line(bgr, c, frame_center, cv::Scalar(255, 0, 255), 1);
  }

  const cv::Scalar text_bg(0, 0, 0);
  const cv::Scalar text_fg(0, 255, 255);
  auto put = [&](const std::string &line, int y) {
    cv::putText(bgr, line, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, text_bg, 3);
    cv::putText(bgr, line, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, text_fg, 1);
  };

  put(std::string("phase: ") + (phase ? phase : "?"), 22);
  if (det) {
    put(std::string(det->label ? det->label : "target") +
            "  u=" + std::to_string(static_cast<int>(det->center.u)) +
            " v=" + std::to_string(static_cast<int>(det->center.v)),
        44);
  } else {
    put("no person (stand in front of the camera)", 44);
  }
  if (angles) {
    put("yaw=" + std::to_string(angles->yaw * 180.0 / kPi) + " deg  pitch=" +
            std::to_string(angles->pitch * 180.0 / kPi) + " deg",
        66);
  }
  if (stats && stats->rejected_border > 0) {
    put("ignored " + std::to_string(stats->rejected_border) + " border/wall blob(s)", 88);
  }
  if (note && note[0] != '\0') {
    put(note, 110);
  }
}

}  // namespace proud_up
