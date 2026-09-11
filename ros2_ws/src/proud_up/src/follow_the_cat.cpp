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

// Turn an OpenCV rectangle into the common PersonDetection the node uses.
//
// head_frac chooses the aim pixel along the box height:
//   0.0 = top edge, 0.5 = middle, 1.0 = bottom.
// For a YOLO person box we use ~0.45 (torso). For a face we use 0.50
// (middle of the face). The contour is just the four corners, so the
// debug overlay can draw a green rectangle.
PersonDetection box_to_detection(const cv::Rect &r, const char *label, double head_frac) {
  PersonDetection det;
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
  // LBP is much faster than Haar on this CPU. Haar alt2 is only the fallback
  // if the LBP xml files are missing.
  ok_ = face_.load(lbp("lbpcascade_frontalface.xml")) ||
        face_.load(lbp("lbpcascade_frontalface_improved.xml")) ||
        face_.load(haar("haarcascade_frontalface_alt2.xml"));

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

  // Ultralytics YOLOv8 ONNX output is [1, 84, N]:
  //   4 box numbers (centre x, centre y, width, height) plus 80 COCO class
  //   scores, all in the stretched 320×320 blob. Class 0 is "person".
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
    // row[4] is the COCO "person" score. We do not require person to beat
    // "chair" or "couch" on the same candidate. A seated person often
    // loses that comparison, and we would miss them.
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
    // Stay on the same body. Picking the largest box each frame walked
    // the camera onto a window curtain and left it stuck there.
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

std::optional<PersonDetection> HumanDetector::detect_face_in(const cv::Mat &bgr,
                                                             const cv::Rect &roi) {
  if (!ok_ || bgr.empty()) {
    return std::nullopt;
  }
  cv::Rect region = roi;
  if (region.width <= 0 || region.height <= 0) {
    region = cv::Rect(0, 0, bgr.cols, bgr.rows);
  } else {
    // Search the upper 65% of the person box, with a little padding, for
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

std::optional<PersonDetection> HumanDetector::detect(const cv::Mat &bgr) {
  const auto t0 = std::chrono::steady_clock::now();
  auto finish = [&](std::optional<PersonDetection> out) {
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
      // Aim at the torso, not the top of the YOLO box. A small head_frac
      // (for example 0.12) pointed the camera at the curtain and the
      // lock-on-centre test then treated that as success.
      const double frac =
          (cfg_.person_head_frac > 0.05 && cfg_.person_head_frac < 0.9) ? cfg_.person_head_frac
                                                                       : 0.45;
      return finish(box_to_detection(*box, "person", frac));
    }
  }

  // Close-up facing the camera with no YOLO box (or no ONNX file loaded).
  if (auto face = detect_face_in(bgr, cv::Rect())) {
    return finish(face);
  }

  return finish(std::nullopt);
}

// ---------------------------------------------------------------------------
// Camera internals
// ---------------------------------------------------------------------------

CameraIntrinsics fallback_intrinsics() {
  // peripherals/config/camera_info.yaml — USB camera numbers, not a promise
  // about Aurora resolution_mode_index=2. Prefer live camera_info whenever
  // it exists.
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
  // Normalizing makes this a unit direction so yaw and pitch do not
  // depend on that arbitrary scale.
  return Eigen::Vector3d(X, Y, 1.0).normalized();
}

// ---------------------------------------------------------------------------
// Yaw and pitch
// ---------------------------------------------------------------------------

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
  // Eigen multiplies on the left: (q_yaw * q_pitch) * v applies pitch
  // first, then yaw. That matches "tilt the lens, then pan". Hardware
  // gets the two scalar angles, not this quaternion; we only log it.
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  return q;
}

// ---------------------------------------------------------------------------
// Servo pulses
// ---------------------------------------------------------------------------

float clamp_pulse(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

float clamp_delta(float value, float max_abs) {
  const float cap = std::abs(max_abs);
  return std::max(-cap, std::min(cap, value));
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

bool near_optical_axis(const GazeAngles &angles, double yaw_tol, double pitch_tol) {
  return std::abs(angles.yaw) <= yaw_tol && std::abs(angles.pitch) <= pitch_tol;
}

void draw_debug_overlay(cv::Mat &bgr, const std::optional<PersonDetection> &det,
                        const char *phase, const GazeAngles *angles, const char *note) {
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
    put(std::string(det->label ? det->label : "person") +
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
  if (note && note[0] != '\0') {
    put(note, 110);
  }
}

}  // namespace proud_up
