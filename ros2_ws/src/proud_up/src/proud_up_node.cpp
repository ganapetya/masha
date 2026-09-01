#include <chrono>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "proud_up/camera_frame.hpp"
#include "proud_up/load_dotenv.hpp"
#include "proud_up/mailer.hpp"
#include "proud_up/pose_commander.hpp"

using namespace std::chrono_literals;

namespace proud_up {
namespace {

std::string getenv_str(const char *name) {
  const char *value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

std::string first_nonempty(const std::string &param, const char *env_name) {
  if (!param.empty()) {
    return param;
  }
  return getenv_str(env_name);
}

std::string hostname_or(const std::string &fallback) {
  char buf[256];
  if (gethostname(buf, sizeof(buf)) == 0) {
    buf[sizeof(buf) - 1] = '\0';
    return std::string(buf);
  }
  return fallback;
}

enum class Phase {
  WaitReady,
  CommandPose,
  Settle,
  Capture,
  Deliver,
  Done,
  Failed
};

const char *phase_name(Phase phase) {
  switch (phase) {
    case Phase::WaitReady:
      return "wait_ready";
    case Phase::CommandPose:
      return "command_pose";
    case Phase::Settle:
      return "settle";
    case Phase::Capture:
      return "capture";
    case Phase::Deliver:
      return "deliver";
    case Phase::Done:
      return "done";
    case Phase::Failed:
      return "failed";
  }
  return "unknown";
}

}  // namespace

class ProudUpNode : public rclcpp::Node {
 public:
  ProudUpNode() : Node("proud_up_node") {
    const auto env_file = load_dotenv();
    if (!env_file.empty()) {
      RCLCPP_INFO(get_logger(), "loaded env from %s", env_file.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  "no .env found; SMTP uses parameters / process environment only");
    }

    image_topic_ = declare_parameter<std::string>("image_topic", "/depth_cam/rgb/image_raw");
    servo_topic_ = declare_parameter<std::string>("servo_topic", "servo_controller");
    controller_ready_service_ =
        declare_parameter<std::string>("controller_ready_service", "/controller_manager/init_finish");
    init_pose_ready_service_ =
        declare_parameter<std::string>("init_pose_ready_service", "/init_pose/init_finish");
    wait_for_ready_ = declare_parameter<bool>("wait_for_ready", true);
    run_on_start_ = declare_parameter<bool>("run_on_start", true);
    settle_seconds_ = declare_parameter<double>("settle_seconds", 2.0);
    ready_timeout_seconds_ = declare_parameter<double>("ready_timeout_seconds", 180.0);
    capture_timeout_seconds_ = declare_parameter<double>("capture_timeout_seconds", 30.0);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 90);
    snapshot_path_ = declare_parameter<std::string>("snapshot_path", "/tmp/masha-up.jpg");
    image_qos_reliable_ = declare_parameter<bool>("image_qos_reliable", false);

    arm_.id19 = static_cast<float>(declare_parameter<double>("arm_id19", 500.0));
    arm_.id20 = static_cast<float>(declare_parameter<double>("arm_id20", 810.0));
    arm_.id21 = static_cast<float>(declare_parameter<double>("arm_id21", 180.0));
    arm_.id22 = static_cast<float>(declare_parameter<double>("arm_id22", 150.0));
    arm_.id23 = static_cast<float>(declare_parameter<double>("arm_id23", 500.0));
    arm_.id24 = static_cast<float>(declare_parameter<double>("arm_id24", 500.0));
    arm_.duration_s = declare_parameter<double>("arm_duration", 1.5);

    SmtpConfig smtp;
    smtp.host = first_nonempty(declare_parameter<std::string>("smtp_host", ""), "MASHA_SMTP_HOST");
    smtp.port = declare_parameter<int>("smtp_port", 587);
    const auto port_env = getenv_str("MASHA_SMTP_PORT");
    if (!port_env.empty()) {
      smtp.port = std::stoi(port_env);
    }
    smtp.user = first_nonempty(declare_parameter<std::string>("smtp_user", ""), "MASHA_SMTP_USER");
    smtp.password =
        first_nonempty(declare_parameter<std::string>("smtp_password", ""), "MASHA_SMTP_PASSWORD");
    smtp.from = first_nonempty(declare_parameter<std::string>("smtp_from", ""), "MASHA_SMTP_FROM");
    smtp.to = first_nonempty(declare_parameter<std::string>("smtp_to", ""), "MASHA_SMTP_TO");
    smtp.encryption =
        first_nonempty(declare_parameter<std::string>("smtp_encryption", "starttls"),
                       "MASHA_SMTP_ENCRYPTION");
    smtp.subject = declare_parameter<std::string>("email_subject", "Masha is up");
    smtp.verbose = declare_parameter<bool>("smtp_verbose", false);
    mailer_ = std::make_unique<Mailer>(std::move(smtp));
    if (mailer_->configured()) {
      RCLCPP_INFO(get_logger(), "SMTP ready host=%s port=%d to=%s encryption=%s",
                  mailer_->config().host.c_str(), mailer_->config().port,
                  mailer_->config().to.c_str(), mailer_->config().encryption.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  "SMTP not configured (need smtp_host/smtp_to in .env or MASHA_SMTP_*)");
    }

    servo_pub_ = create_publisher<servo_controller_msgs::msg::ServosPosition>(servo_topic_, 1);

    rclcpp::QoS image_qos(rclcpp::KeepLast(5));
    if (image_qos_reliable_) {
      image_qos.reliable();
    } else {
      image_qos.best_effort();
    }
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, image_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(std::move(msg)); });

    controller_client_ = create_client<std_srvs::srv::Trigger>(controller_ready_service_);
    init_pose_client_ = create_client<std_srvs::srv::Trigger>(init_pose_ready_service_);

    run_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/run",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (phase_ != Phase::Done && phase_ != Phase::Failed && phase_ != Phase::WaitReady) {
            response->success = false;
            response->message = "proud_up already running";
            return;
          }
          start_sequence_locked(/*skip_ready=*/true);
          response->success = true;
          response->message = "re-run started";
        });

    init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/init_finish",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = (phase_ == Phase::Done);
          response->message = phase_name(phase_);
        });

    started_at_ = now();
    timer_ = create_wall_timer(100ms, [this]() { tick(); });

    if (run_on_start_) {
      RCLCPP_INFO(get_logger(),
                  "proud_up waiting for ready (controller=%s init_pose=%s camera=%s)",
                  controller_ready_service_.c_str(), init_pose_ready_service_.c_str(),
                  image_topic_.c_str());
    } else {
      phase_ = Phase::Done;
      RCLCPP_INFO(get_logger(), "proud_up idle; call ~/run to take the boot snapshot");
    }
  }

 private:
  rclcpp::Time now() { return get_clock()->now(); }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_image_ = std::move(msg);
  }

  void start_sequence_locked(bool skip_ready) {
    failed_reason_.clear();
    shared_frame_.reset();
    capture_after_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    phase_started_at_ = now();
    started_at_ = now();
    phase_ = skip_ready ? Phase::CommandPose : Phase::WaitReady;
  }

  void set_phase(Phase next) {
    RCLCPP_INFO(get_logger(), "proud_up: %s -> %s", phase_name(phase_), phase_name(next));
    phase_ = next;
    phase_started_at_ = now();
  }

  bool timed_out(double limit_s) {
    return (now() - phase_started_at_).seconds() > limit_s;
  }

  void fail(const std::string &reason) {
    failed_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    set_phase(Phase::Failed);
  }

  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    switch (phase_) {
      case Phase::WaitReady:
        tick_wait_ready();
        break;
      case Phase::CommandPose:
        tick_command_pose();
        break;
      case Phase::Settle:
        tick_settle();
        break;
      case Phase::Capture:
        tick_capture();
        break;
      case Phase::Deliver:
        tick_deliver();
        break;
      case Phase::Done:
      case Phase::Failed:
        break;
    }
  }

  void tick_wait_ready() {
    if (!wait_for_ready_) {
      set_phase(Phase::CommandPose);
      return;
    }
    const bool controller_ready = controller_client_->service_is_ready();
    const bool init_pose_ready = init_pose_client_->service_is_ready();
    const bool camera_ready = static_cast<bool>(latest_image_);
    if (controller_ready && init_pose_ready && camera_ready) {
      RCLCPP_INFO(get_logger(), "Masha is ready; commanding camera-forward arm pose");
      set_phase(Phase::CommandPose);
      return;
    }
    if ((now() - started_at_).seconds() > ready_timeout_seconds_) {
      std::ostringstream oss;
      oss << "timeout waiting for ready (controller=" << controller_ready
          << " init_pose=" << init_pose_ready << " camera=" << camera_ready << ")";
      fail(oss.str());
    }
  }

  void tick_command_pose() {
    servo_pub_->publish(make_arm_command(arm_));
    capture_after_ = now() + rclcpp::Duration::from_seconds(arm_.duration_s + settle_seconds_);
    set_phase(Phase::Settle);
  }

  void tick_settle() {
    if (now() >= capture_after_) {
      set_phase(Phase::Capture);
    }
  }

  void tick_capture() {
    if (!latest_image_) {
      if (timed_out(capture_timeout_seconds_)) {
        fail("timeout waiting for camera frame after pose");
      }
      return;
    }
    const auto stamp = rclcpp::Time(latest_image_->header.stamp);
    if (stamp.nanoseconds() != 0 && stamp < capture_after_) {
      if (timed_out(capture_timeout_seconds_)) {
        fail("timeout waiting for a frame newer than the proud pose");
      }
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(latest_image_, "bgr8");
    } catch (const cv_bridge::Exception &ex) {
      fail(std::string("cv_bridge: ") + ex.what());
      return;
    }
    if (!cv_ptr || cv_ptr->image.empty()) {
      fail("empty camera image");
      return;
    }

    // Unique ownership of the encoded JPEG; shared only when disk + SMTP both need it.
    std::unique_ptr<CameraFrame> unique_frame = encode_jpeg(cv_ptr->image, jpeg_quality_);
    if (!unique_frame) {
      fail("JPEG encode failed");
      return;
    }
    shared_frame_ = std::shared_ptr<const CameraFrame>(std::move(unique_frame));
    RCLCPP_INFO(get_logger(), "captured %dx%d jpeg (%zu bytes) at %s", shared_frame_->width,
                shared_frame_->height, shared_frame_->jpeg.size(),
                shared_frame_->captured_at_utc.c_str());
    set_phase(Phase::Deliver);
  }

  void tick_deliver() {
    if (!shared_frame_) {
      fail("internal error: missing shared frame");
      return;
    }

    if (!save_jpeg(shared_frame_, snapshot_path_)) {
      RCLCPP_WARN(get_logger(), "could not write snapshot to %s", snapshot_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "wrote snapshot %s", snapshot_path_.c_str());
    }

    if (!mailer_->configured()) {
      RCLCPP_WARN(get_logger(),
                  "SMTP not configured yet (set smtp_host/smtp_to or MASHA_SMTP_*). "
                  "Snapshot kept at %s; email skipped.",
                  snapshot_path_.c_str());
      set_phase(Phase::Done);
      return;
    }

    std::ostringstream body;
    body << "Masha is up.\n"
         << "host: " << hostname_or("masha") << "\n"
         << "time: " << shared_frame_->captured_at_utc << "\n"
         << "image: " << shared_frame_->width << "x" << shared_frame_->height << "\n";
    SmtpConfig cfg = mailer_->config();
    cfg.body = body.str();
    Mailer sender(std::move(cfg));

    std::string error;
    if (!sender.send(shared_frame_, error)) {
      RCLCPP_ERROR(get_logger(), "SMTP send failed: %s", error.c_str());
      // Photo is on disk; node still reports done so boot is not stuck.
    } else {
      RCLCPP_INFO(get_logger(), "emailed snapshot with subject '%s'",
                  mailer_->config().subject.c_str());
    }
    set_phase(Phase::Done);
  }

  std::string image_topic_;
  std::string servo_topic_;
  std::string controller_ready_service_;
  std::string init_pose_ready_service_;
  std::string snapshot_path_;
  bool wait_for_ready_{true};
  bool run_on_start_{true};
  bool image_qos_reliable_{false};
  double settle_seconds_{2.0};
  double ready_timeout_seconds_{180.0};
  double capture_timeout_seconds_{30.0};
  int jpeg_quality_{90};
  ArmPulses arm_;

  std::unique_ptr<Mailer> mailer_;
  rclcpp::Publisher<servo_controller_msgs::msg::ServosPosition>::SharedPtr servo_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  Phase phase_{Phase::WaitReady};
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time capture_after_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::shared_ptr<const CameraFrame> shared_frame_;
  std::string failed_reason_;
};

}  // namespace proud_up

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<proud_up::ProudUpNode>());
  rclcpp::shutdown();
  return 0;
}
