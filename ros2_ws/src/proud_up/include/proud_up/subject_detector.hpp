#pragma once

// Shared subject detector. No ROS.
//
// Follow-the-cat and the hunter both call SubjectDetector. The ONNX file
// is yolov8n.onnx. coco_class picks one COCO column: 0 is a person, 15 is
// a cat. The same forward pass scores every class.
//
// The LBP face cascade is a second, cheaper pass, and only for a person
// looking at the lens. It is optional. A cat turns it off, because a face
// is not a cat.
//
// This header stops at a pixel. Turning that pixel into a camera ray and
// a servo pulse is follow_the_cat.hpp. The hunter turns the same pixel
// into a base_link pose in the node.

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>

namespace proud_up {

// Pixel coordinates in the camera image. Origin is the top-left corner.
// u grows right, v grows down. Units are pixels, not metres.
struct Pixel {
  double u{0.0};
  double v{0.0};
};

// One box the rest of the robot can use. YOLO and the face cascade fill
// the same struct. After this returns, the caller only needs a pixel to
// aim at, a rectangle to draw, and a short label.
//
// std::optional<SubjectDetection> is the "did we see the subject?" result:
//   has a value  →  there is a box this frame
//   empty        →  a miss
struct SubjectDetection {
  Pixel center;                       // the pixel we aim at
  std::vector<cv::Point> contour;     // rectangle corners, for the debug image
  std::vector<cv::Point> approx;      // same as contour for a YOLO / face box
  double area{0.0};                   // box width * height, in px^2
  double aspect{0.0};                 // width / height
  const char *label{"person"};        // "person", "cat", "face", or "coast"
};

// person_head_frac chooses the aim pixel inside the YOLO box. 0.0 is the
// top edge, 0.5 is the middle, 1.0 is the bottom. 0.45 aims at a person's
// torso. Aiming too high (for example 0.12) sends the camera up toward the
// ceiling and it stays there.
//
// The person_* field names are the ROS parameter names in
// follow_the_cat.yaml and masha_hunter.yaml. The detector methods say
// "subject" because the same net serves more than one COCO class.
struct SubjectDetectConfig {
  std::string cascade_dir{"/usr/share/opencv4/haarcascades"};
  std::string lbp_dir{"/usr/share/opencv4/lbpcascades"};
  std::string person_onnx;  // empty = skip YOLO (tests, or face-only)
  double image_scale{0.45};  // LBP face runs on a smaller grey image
  double scale_factor{1.2};
  int min_neighbors{3};
  int min_face{24};  // pixels in the original image, not the scaled copy
  double person_conf{0.45};
  double person_iou{0.45};
  int person_imgsz{320};
  bool dnn_cuda{true};
  bool enable_face_refine{false};
  double person_head_frac{0.45};
  double max_box_frac{0.40};     // a box that fills the frame is a wall, not a subject
  double max_aspect{1.05};       // used when require_person_shape is true
  double track_gate_frac{0.22};  // stay on the same box; do not jump to a neighbour
  int coco_class{0};             // 0 = person, 15 = cat. Same ONNX, different column.
  bool require_person_shape{true};
  bool enable_face_fallback{true};
};

// YOLOv8 row: 4 box numbers, then COCO scores. True when `cls` is present
// and no other class scores strictly higher. Cat detection requires this.
// Class 0 does not call it (a seated person often loses to "chair").
inline bool coco_class_is_best(const float *row, int cols, int cls, int n_classes = 80) {
  if (row == nullptr || cls < 0 || cols < 5 || (4 + cls) >= cols) {
    return false;
  }
  const float mine = row[4 + cls];
  int n = n_classes;
  if (n > cols - 4) {
    n = cols - 4;
  }
  for (int c = 0; c < n; ++c) {
    if (c == cls) {
      continue;
    }
    if (row[4 + c] > mine) {
      return false;
    }
  }
  return true;
}

class SubjectDetector {
 public:
  explicit SubjectDetector(const SubjectDetectConfig &cfg = {});
  ~SubjectDetector();
  bool ok() const { return ok_ || subject_ok_; }
  // True when yolov8n.onnx loaded. The face cascade can still be ok alone.
  bool subject_ok() const { return subject_ok_; }
  bool using_cuda() const { return using_cuda_; }
  double last_detect_ms() const { return last_detect_ms_; }
  void reset_track();
  // Copy runtime scalars (thresholds, gates). Does not reload ONNX or cascades.
  void apply_runtime_cfg(const SubjectDetectConfig &cfg);
  std::optional<SubjectDetection> detect(const cv::Mat &bgr);

 private:
  std::optional<cv::Rect> detect_subject_box(const cv::Mat &bgr);
  std::optional<SubjectDetection> detect_face_in(const cv::Mat &bgr, const cv::Rect &roi);
  void warmup_subject_net();
  static bool looks_like_subject(const cv::Rect &r, int width, int height,
                                 const SubjectDetectConfig &cfg);

  struct SubjectNet;
  SubjectDetectConfig cfg_;
  cv::CascadeClassifier face_;
  std::unique_ptr<SubjectNet> subject_;
  cv::Rect last_subject_;
  bool have_last_subject_{false};
  bool ok_{false};
  bool subject_ok_{false};
  bool using_cuda_{false};
  double last_detect_ms_{0.0};
};

}  // namespace proud_up
