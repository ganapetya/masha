#pragma once

#include <memory>
#include <string>

#include "proud_up/camera_frame.hpp"

namespace proud_up {

struct SmtpConfig {
  std::string host;
  int port{587};
  std::string user;
  std::string password;
  std::string from;
  std::string to;  // comma-separated
  std::string encryption{"starttls"};  // starttls | ssl | none
  std::string subject{"Masha is up"};
  std::string body;
  bool verbose{false};
};

class Mailer {
 public:
  explicit Mailer(SmtpConfig config);

  bool configured() const;
  // Shared frame: mailer does not take exclusive ownership of the JPEG.
  bool send(const std::shared_ptr<const CameraFrame> &frame, std::string &error) const;

  const SmtpConfig &config() const { return config_; }

 private:
  SmtpConfig config_;
};

}  // namespace proud_up
