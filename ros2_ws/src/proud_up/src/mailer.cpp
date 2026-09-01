#include "proud_up/mailer.hpp"

#include <sstream>
#include <utility>
#include <vector>

#include <curl/curl.h>

namespace proud_up {
namespace {

struct CurlGlobal {
  CurlGlobal() { curl_global_init(CURL_GLOBAL_DEFAULT); }
  ~CurlGlobal() { curl_global_cleanup(); }
};

void ensure_curl_global() {
  static CurlGlobal global;
  static_cast<void>(global);
}

struct CurlHandle {
  CURL *handle{nullptr};
  CurlHandle() : handle(curl_easy_init()) {}
  ~CurlHandle() {
    if (handle) {
      curl_easy_cleanup(handle);
    }
  }
  CurlHandle(const CurlHandle &) = delete;
  CurlHandle &operator=(const CurlHandle &) = delete;
};

struct CurlSlist {
  curl_slist *list{nullptr};
  ~CurlSlist() {
    if (list) {
      curl_slist_free_all(list);
    }
  }
  void append(const std::string &value) { list = curl_slist_append(list, value.c_str()); }
};

struct CurlMime {
  curl_mime *mime{nullptr};
  explicit CurlMime(CURL *curl) : mime(curl_mime_init(curl)) {}
  ~CurlMime() {
    if (mime) {
      curl_mime_free(mime);
    }
  }
  CurlMime(const CurlMime &) = delete;
  CurlMime &operator=(const CurlMime &) = delete;
};

std::string angle_addr(std::string addr) {
  while (!addr.empty() && (addr.front() == ' ' || addr.front() == '\t')) {
    addr.erase(addr.begin());
  }
  while (!addr.empty() && (addr.back() == ' ' || addr.back() == '\t')) {
    addr.pop_back();
  }
  if (addr.empty()) {
    return addr;
  }
  if (addr.front() == '<') {
    return addr;
  }
  return "<" + addr + ">";
}

std::vector<std::string> split_recipients(const std::string &to) {
  std::vector<std::string> out;
  std::stringstream ss(to);
  std::string item;
  while (std::getline(ss, item, ',')) {
    auto wrapped = angle_addr(item);
    if (!wrapped.empty()) {
      out.push_back(std::move(wrapped));
    }
  }
  return out;
}

std::string smtp_url(const SmtpConfig &config) {
  const std::string scheme =
      (config.encryption == "ssl") ? "smtps://" : "smtp://";
  return scheme + config.host + ":" + std::to_string(config.port);
}

}  // namespace

Mailer::Mailer(SmtpConfig config) : config_(std::move(config)) {
  ensure_curl_global();
  if (config_.from.empty()) {
    config_.from = config_.user;
  }
}

bool Mailer::configured() const {
  return !config_.host.empty() && !config_.to.empty();
}

bool Mailer::send(const std::shared_ptr<const CameraFrame> &frame, std::string &error) const {
  error.clear();
  if (!configured()) {
    error = "SMTP is not configured (smtp_host / smtp_to empty)";
    return false;
  }
  if (!frame || frame->jpeg.empty()) {
    error = "no JPEG frame to attach";
    return false;
  }

  CurlHandle curl;
  if (!curl.handle) {
    error = "curl_easy_init failed";
    return false;
  }

  const auto recipients = split_recipients(config_.to);
  if (recipients.empty()) {
    error = "no SMTP recipients";
    return false;
  }

  const std::string from = angle_addr(config_.from);
  curl_easy_setopt(curl.handle, CURLOPT_URL, smtp_url(config_).c_str());
  curl_easy_setopt(curl.handle, CURLOPT_MAIL_FROM, from.c_str());

  CurlSlist rcpt;
  for (const auto &addr : recipients) {
    rcpt.append(addr);
  }
  curl_easy_setopt(curl.handle, CURLOPT_MAIL_RCPT, rcpt.list);

  if (!config_.user.empty()) {
    curl_easy_setopt(curl.handle, CURLOPT_USERNAME, config_.user.c_str());
  }
  if (!config_.password.empty()) {
    curl_easy_setopt(curl.handle, CURLOPT_PASSWORD, config_.password.c_str());
  }

  if (config_.encryption == "starttls" || config_.encryption == "ssl") {
    curl_easy_setopt(curl.handle, CURLOPT_USE_SSL, static_cast<long>(CURLUSESSL_ALL));
  }

  curl_easy_setopt(curl.handle, CURLOPT_UPLOAD, 0L);
  curl_easy_setopt(curl.handle, CURLOPT_VERBOSE, config_.verbose ? 1L : 0L);
  curl_easy_setopt(curl.handle, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl.handle, CURLOPT_TIMEOUT, 20L);

  CurlSlist headers;
  headers.append("From: " + from);
  headers.append("To: " + config_.to);
  headers.append("Subject: " + config_.subject);
  curl_easy_setopt(curl.handle, CURLOPT_HTTPHEADER, headers.list);

  CurlMime mime(curl.handle);
  if (!mime.mime) {
    error = "curl_mime_init failed";
    return false;
  }

  curl_mimepart *text = curl_mime_addpart(mime.mime);
  const std::string body =
      config_.body.empty() ? std::string("Masha is up.\n") : config_.body;
  curl_mime_data(text, body.c_str(), CURL_ZERO_TERMINATED);
  curl_mime_type(text, "text/plain; charset=utf-8");

  curl_mimepart *image = curl_mime_addpart(mime.mime);
  curl_mime_data(image, reinterpret_cast<const char *>(frame->jpeg.data()),
                 frame->jpeg.size());
  curl_mime_filename(image, "masha-up.jpg");
  curl_mime_type(image, "image/jpeg");
  curl_mime_encoder(image, "base64");

  curl_easy_setopt(curl.handle, CURLOPT_MIMEPOST, mime.mime);

  const CURLcode rc = curl_easy_perform(curl.handle);
  if (rc != CURLE_OK) {
    error = curl_easy_strerror(rc);
    return false;
  }
  return true;
}

}  // namespace proud_up
