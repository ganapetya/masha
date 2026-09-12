# Sprint 3 — Gemini Notebook source (verbatim)

**Pasted from:** `~/host-clipboard.txt`  
**Date on Masha:** 2026-09-12  
**Status:** syllabus as Gemini wrote it. Do **not** implement this text as-is. The executable plan is [`quarter-01-sprint-03-plan.md`](quarter-01-sprint-03-plan.md).

The block below is unchanged from the clipboard.

---

Sprint 3 Learning Plan: Voice-Activated Gait & Interaction for Hexapod 'Masha'

1. Sprint Overview and Objectives

The objective of Sprint 3 is the architectural integration of offline Automatic Speech Recognition (ASR) with the Masha hexapod's motion control stack. This sprint focuses on moving beyond standard teleoperation into autonomous interaction logic. The goal is to implement a high-level state machine that identifies a specific master query, provides an audible response via PulseAudio, and triggers a complex, stability-aware dance sequence.

Definition of Done (DoD)

* Vocal Trigger Identification: Successful matching of the offline ASR string "Masha, who is your master?" using the sherpa-onnx node.
* Audio Synthesis Response: Audible output of "Peter Kovgan is my master" through the paired Bluetooth speaker.
* Safety Interlock: Automated clearing of the controller gait buffer via a zero-velocity geometry_msgs/msg/Twist publication to /cmd_vel prior to any Action Set execution.
* Action Set Execution: Successful invocation of the /RunActionSet service to playback a custom .d6a dance sequence.
* Dynamic Stability: Verification that the Zero Moment Point (ZMP) remains within the support polygon throughout the rhythmic sequence.

2. Day 1: Hexapod Locomotion Theory & Stability

As a hexapod, Masha’s stability and maneuverability constraints differ significantly from holonomic 4-wheel mecanum-drive systems like the Yahboom X3. While the X3 base allows for instantaneous lateral translation, Masha’s tripod gait is natively non-holonomic unless specific lateral Inverse Kinematics (IK) steps are explicitly programmed.

Stability Foundations

* Support Polygons: Static stability is maintained by ensuring the projection of Masha’s Center of Mass (CoM) remains within the polygon formed by the grounded feet.
* Zero Moment Point (ZMP): For dynamic "dance" gaits, the point where the sum of horizontal inertial and gravitational forces equals zero must stay within the support polygon to prevent tipping during rhythmic shifts.

Gait Comparison

Gait Type	Leg Coordination	Architectural Trade-offs
Tripod Gait	Legs (1,3,5) and (2,4,6) alternate.	High velocity; smaller support triangle; non-holonomic by default.
Ripple Gait	Wave-like sequence (typically 1-2 legs lifted).	Maximum stability; large support polygon (4–5 grounded); lower velocity.

3. Day 2: Kinematics and Transformation (TF) Trees

Masha’s architecture models the 3-DOF (Degree of Freedom) leg segments as a hierarchical chain. Accurate modeling of these transformations is essential for calculating the position of the Tibia tip (foot) relative to the body center (base_link).

Homogeneous Transformation Matrices (T_{4 \times 4})

Leg tip positions are calculated using 4 \times 4 matrices that encapsulate both the rotation (R_{3 \times 3}) of the joints and the translation (\Delta p_{3 \times 1}) of the segments:

T_{4 \times 4} = \begin{bmatrix} R_{3 \times 3} & \Delta p_{3 \times 1} \\ 0_{1 \times 3} & 1 \end{bmatrix}

The TF Tree Chain

For each of Masha’s six legs, the transform chain is modeled as: world -> odom -> base_link -> coxa (Link 1) -> femur (Link 2) -> tibia (Link 3).

Forward Kinematics (FK) Checklist

To verify a single 3-DOF leg, apply the T_n^0 product formula sequence:

* [ ] Joint 1 (\theta_1): Coxa rotation (horizontal plane).
* [ ] Joint 2 (\theta_2): Femur pitch (vertical plane).
* [ ] Joint 3 (\theta_3): Tibia pitch (vertical plane).
* [ ] Product: T_3^0 = T_{coxa}^0(\theta_1) T_{femur}^{coxa}(\theta_2) T_{tibia}^{femur}(\theta_3).
* [ ] Advanced Note: Calculate the Jacobian (J) to map joint velocities \dot{\theta} to Cartesian velocities. Monitor det(J) to avoid Kinematic Singularities during the dance sequence, where joint speeds could theoretically approach infinity.

4. Day 3: ROS 2 C++ Voice Recognition with sherpa-onnx

Masha’s voice interaction relies on the sherpa-onnx node for offline Automatic Speech Recognition. This node should be integrated within the peripherals package (augmenting the existing xf_mic_asr_offline logic) and configured via the .typerc file.

Configuration

In ros2_ws/.typerc, ensure:

* ASR_OFFLINE=true
* MIC_TYPE=usb_array

Master Recognition Logic

The following C++ logic must be implemented to identify the master query:

#include "std_msgs/msg/string.hpp"
// Architecture requires a dedicated interaction node

class MashaInteractionNode : public rclcpp::Node {
public:
    MashaInteractionNode() : Node("masha_interaction_logic") {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/voice_asr_result", 10, std::bind(&MashaInteractionNode::handle_speech, this, _1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void handle_speech(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.find("who is your master") != std::string::npos) {
            this->execute_master_sequence();
        }
    }
    // Implementation continues in Day 5/6...
};


5. Day 4: Utilizing the ActionSet Editor for Sequence Serialization

Masha’s complex "dance" is not a standard gait but a serialized sequence of joint configurations defined as an Action Group.

* Tooling: Use software/actionset_editor (Qt-based) to define joint angles (\theta_0, \dots, \theta_n).
* Inverse Kinematics: The kinematics package (utilizing kinematics.so) calculates the required angles for precise foot placements.
* Serialization: Action sets must be saved to the hardcoded runtime path: /home/ubuntu/software/actionset_editor/ActionGroups/.
* File Format: Ensure the dance is saved as a .d6a file for compatibility with the servo_controller node.

6. Day 5: State Machine Integration & Logic Flow

The interaction follows a strict state-based logic to ensure robot safety and prevent conflicting motion commands.

System States

1. LISTEN: sherpa-onnx node continuously monitors the USB mic array.
2. IDENTIFY: String matching against "Masha, who is your master?".
3. SAFETY INTERLOCK: The node publishes an empty Twist to /cmd_vel. This is critical to clear the controller node’s gait buffer.
4. VOICE RESPOND: Execute a system call to paplay (PulseAudio Play) to the Bluetooth speaker: "Peter Kovgan is my master."
5. EXECUTE MOTION: Call the /RunActionSet service (provided by servo_controller) with the dance file name.
6. RESET: Return to IDLE state.

7. Day 6: Master Recognition and Audio Response Programming

To provide the voice response, Masha utilizes the Jetson Orin NX’s PulseAudio system paired with the Bluetooth speaker.

Response Implementation

* Audio Playback: Use a system() call or a dedicated bark audio node to trigger the master's name: system("paplay /home/ubuntu/ros2_ws/src/peripherals/sounds/master_response.wav");
* Service Integration: The node must act as a Service Client for yahboomcar_msgs::srv::RunActionSet.
* Data Fidelity: The response string "Peter Kovgan is my master" is a hard requirement based on the system's primary user identification.

8. Day 7: Testing, Calibration, and Review

Final deployment is performed directly on the Jetson Orin NX hardware.

Deployment Checklist

1. Incremental Build: colcon build --symlink-install --packages-select peripherals servo_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
2. Environment Source: source install/local_setup.zsh
3. Calibration: Verify servo torque limits. Rhythmic dance gaits can cause high current spikes; verify that the 3S LiPo voltage (monitored on /voltage) remains stable.

Troubleshooting

Symptom	Probable Cause	Resolution
Voice Not Detected	Mic target in .typerc misconfigured.	Verify PulseAudio sources with pactl list sources.
Gait Instability	ZMP shift too rapid for Coxa servos.	Reduce playback speed in ActionSet Editor.
Service Timeout	/dev/ttyACM0 disconnected.	Verify STM32 connection (ros_robot_controller). Note: Do not use ttyUSB1 (X3 chassis).
No Audio Output	Bluetooth speaker power-save mode.	Re-pair speaker via PulseAudio/BlueZ.
