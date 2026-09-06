# 📓 Learning Plan: Quarter 1 Week 2 (Sprint 2) - Version 2
*Project Focus: Building the `follow_the_cat_node` (White Rectangle Tracking)*

Welcome to Sprint 2, Version 2! To ground your study of 3D rotations, Eigen geometry, and C++ memory mechanics in a rewarding, physical application, we are pivoting our exercises to build a standalone ROS 2 C++ node: **`follow_the_cat_node`**.

### The Scenario
Instead of starting with complex neural networks, we will use a **white rectangle** (a simple sheet of paper or white card) as our stand-in "cat". When you show the white rectangle to Masha:
1. **Find & Target**: Masha detects the rectangle's 2D image coordinates, converts them to a 3D direction vector using camera optics, and uses 3D rotations to center her camera directly on it.
2. **Track**: The neck pan (Servo 19) and tilt (Servo 23) servos dynamically track the rectangle as you move it around.
3. **Approach**: After a 2-second tracking pause, Masha commands her leg controller (`cmd_vel`) to walk several steps directly forward toward the rectangle's location.

---

## 📈 My Progress Tracker

- [ ] **Quarter 1: Foundations of Spatial Mechanics & Edge CV**
  - [x] **Week 1: The Coordinate & Memory Foundation** ── *Completed!*
  - [ ] **Week 2: Rotations in 3D Space ($SO(3)$ and Quaternions)** ── 🎯 *CURRENT SPRINT*
  - [ ] Week 3: Camera Extrinsics & Camera Matrix Projection
  - [ ] Week 4: Multi-Threaded Image Acquisition & Ring Buffers
  - [ ] Week 5: OpenCV Image Segmentation & Color Space Invariance
  - [ ] Week 6: Deep Learning Edge Deployment (ONNX Runtime on Jetson)
  - [ ] Week 7: Cat Detection & Multi-threaded Inter-Process Communication
  - [ ] Week 8: Behavioral Trees for Multi-agent Tracking (Masha meets Saveli)
  - [ ] Week 9: Phase 1 Capstone: Autonomous Cat Spotting & Tracking

---

## 🎯 Sprint 2 Goals

1. **The Math**: Understand coordinate transformations between the **Camera Optical Frame** and the **Robot Body Base Frame**. Master the use of Unit Quaternions (`Eigen::Quaterniond`) and Axis-Angle (`Eigen::AngleAxisd`) to rotate coordinate systems cleanly.
2. **The C++ Mechanics**: Expand your knowledge of:
   * **Thread Safety**: `std::mutex` and `std::lock_guard` to protect shared state between subscriber callbacks and the timer-driven walking state machine.
   * **OpenCV C++ API**: Working with matrices (`cv::Mat`), converting ROS images via `cv_bridge`, color-thresholding, and contour isolation.
   * **Smart Pointers**: Passing image buffers around using `std::unique_ptr` and `std::shared_ptr`.
3. **ROS 2 Concepts**: Writing a custom subscription to `sensor_msgs/msg/Image`, publishing joint position arrays, and using standard Twist velocity commands (`geometry_msgs/msg/Twist`) to make Masha walk.

---

## 📅 Daily Micro-Lessons (1 hr/day)

### 🗓️ Day 1: Capturing and Filtering the Image (ROS 2 & OpenCV)
* **Concept**: Subscribing to Masha's Deptrum RGB camera topic and thresholding for the white card.
* **C++ Task**: Set up your subscription callback using `cv_bridge::toCvCopy` to convert incoming ROS messages into a `cv::Mat`.
* **C++ Focus**:
  * Implement an OpenCV pipeline: `cv::cvtColor` to convert to Grayscale, followed by `cv::threshold` to isolate pure white pixels.
  * Use `cv::findContours` to isolate shapes, loop through contours, and find the largest rectangular box.
  * Save the 2D pixel coordinate `(u, v)` of the box's center.

### 🗓️ Day 2: Transforming Pixels to 3D Rays (Camera Intrinsics)
* **Concept**: A 2D pixel coordinate `(u, v)` represents an infinite 3D line (ray) projecting out of the lens. We need to convert `(u, v)` into a 3D direction vector `(X, Y, Z)` in the camera's optical frame.
* **Math Equation**:
  Given the camera intrinsic values (focal length $f_x, f_y$ and principal point $c_x, c_y$):
  $$X_{cam} = \frac{u - c_x}{f_x}$$
  $$Y_{cam} = \frac{v - c_y}{f_y}$$
  $$Z_{cam} = 1.0 \quad (\text{normalized depth})$$
* **Eigen Task**: Store this 3D vector as an `Eigen::Vector3d` and normalize it to make it a unit direction vector:
  `Eigen::Vector3d target_ray = Eigen::Vector3d(X, Y, 1.0).normalized();`

### 🗓️ Day 3: Calculating Servo Angles (Eigen Geometry)
* **Concept**: Calculate how much Masha's neck servos must rotate to align her lens (pointing along its local Z-axis) with the `target_ray`.
* **Math Task**: Convert the angular offset into Pitch and Yaw angles.
  * **Yaw (Pan - Servo 19)**: $\theta_{yaw} = \text{atan2}(X_{cam}, Z_{cam})$
  * **Pitch (Tilt - Servo 23)**: $\theta_{pitch} = \text{atan2}(-Y_{cam}, \sqrt{X_{cam}^2 + Z_{cam}^2})$
* **Eigen Integration**: Convert these angles into an `Eigen::Quaterniond` representing the new camera head orientation to verify coordinate limits.

### 🗓️ Day 4: State Machine & Thread Safety (C++ Concurrency)
* **Concept**: The camera subscription thread continuously updates the target's position, but the state machine timer executes sequentially at 10Hz to move Masha. We must protect this shared coordinate with a lock.
* **C++ Task**:
  * Define a state enum: `Phase { IDLE, TRACKING, MOVING, DONE }`.
  * Define a shared struct containing the last seen `(u, v)` pixel coordinate and a timestamp.
  * Protect this shared struct using `std::mutex` and `std::lock_guard<std::mutex>` inside both the image callback thread and the timer `tick()` thread to prevent memory corruption/race conditions.

### 🗓️ Day 5: Servo Commands & Gait Execution (ROS 2 Control)
* **Concept**: Publishing physical movements to Masha's hardware.
* **Action 1 (Tracking)**: Publish joint targets to Masha's neck servos:
  * Topic: `/servo_controller` (sending pulses mapped from your calculated `theta_yaw` and `theta_pitch`).
* **Action 2 (Walking)**: If Masha has tracked the card for 2 continuous seconds, transition to `Phase::MOVING`.
  * Publish a forward velocity command to Masha's legs:
    * Topic: `/cmd_vel`
    * Message Type: `geometry_msgs::msg::Twist`
    * Command: `linear.x = 0.05 m/s` for 3 seconds, then stop (`linear.x = 0.0`).

### 🗓️ Day 6: Writing Automated `gtest` Calculations
* **Concept**: Verifying your 3D transformation math offline without needing Masha powered on.
* **C++ Task**: Set up a test suite `test_follow_the_cat.cpp` in Google Test.
* **Test Cases**:
  * Pass a mock pixel in the exact center of the screen `(cx, cy)`; verify your ray solver outputs `(0.0, 0.0, 1.0)`.
  * Pass a mock pixel shifted to the right; verify your calculated `theta_yaw` is a positive angle (turning left/right correctly).

### 🗓️ Day 7: Build, Debug, and Run!
* **Task**:
  * Compile the package: `colcon build --packages-select proud_up`.
  * Launch Masha's core drivers and camera: `ros2 launch masha_bringup robot.launch.py`.
  * Run your node: `ros2 run proud_up follow_the_cat_node`.
  * Hold up a white piece of paper and watch Masha smoothly lock her gaze onto it, pause, and walk toward you!

---

## 💻 C++ & ROS 2 Concurrency Focus (Grok Collaboration)

When you hand this program to Grok to begin development, pay special attention to these technical patterns:

### 1. The Subscriber Thread-Safe Capture Pattern
We avoid copying heavy cv::Mat structures inside our loops. Instead, we swap memory blocks using `std::unique_ptr` or safe references:

```cpp
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

class FollowTheCatNode : public rclcpp::Node {
private:
    std::mutex image_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_msg_ GUARDED_BY(image_mutex_);

    void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_msg_ = msg; // Shared pointer reference count safely incremented
    }
};
```

### 2. Camera Intrinsic Calibration Model
We will hardcode standard defaults for Masha's Deptrum RGB camera, which you can later load from a calibration file:
```cpp
// Average intrinsics for 640x480 resolution
const double fx = 525.0; // Focal length X
const double fy = 525.0; // Focal length Y
const double cx = 320.0; // Center X
const double cy = 240.0; // Center Y
```

***

Let's execute this program! We will build it modularly, day-by-day. Your next step with Grok is to create the template file `src/follow_the_cat_node.cpp` and implement the Day 1 OpenCV thresholding callback.
