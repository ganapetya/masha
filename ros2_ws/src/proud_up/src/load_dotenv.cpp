#include "proud_up/load_dotenv.hpp"

#include <cstdlib>
#include <fstream>
#include <unistd.h>
#include <vector>

namespace proud_up {
namespace {

std::string trim(std::string value) {
  const auto is_space = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\r'; };
  while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

std::string strip_quotes(std::string value) {
  if (value.size() >= 2) {
    const char first = value.front();
    const char last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      return value.substr(1, value.size() - 2);
    }
  }
  return value;
}

bool file_exists(const std::string &path) {
  return !path.empty() && access(path.c_str(), R_OK) == 0;
}

}  // namespace

bool parse_dotenv_line(const std::string &raw, EnvAssignment &out) {
  std::string line = trim(raw);
  if (line.empty() || line.front() == '#') {
    return false;
  }
  if (line.rfind("export ", 0) == 0) {
    line = trim(line.substr(7));
  }
  const auto eq = line.find('=');
  if (eq == std::string::npos || eq == 0) {
    return false;
  }
  out.key = trim(line.substr(0, eq));
  out.value = strip_quotes(trim(line.substr(eq + 1)));
  if (out.key.empty()) {
    return false;
  }
  return true;
}

std::vector<std::string> dotenv_search_paths() {
  std::vector<std::string> paths;
  if (const char *explicit_path = std::getenv("PROUD_UP_ENV_FILE")) {
    if (*explicit_path) {
      paths.emplace_back(explicit_path);
    }
  }
  if (const char *home = std::getenv("HOME")) {
    paths.push_back(std::string(home) + "/ros2_ws/src/proud_up/.env");
  }
  paths.emplace_back("/home/ubuntu/ros2_ws/src/proud_up/.env");
  paths.emplace_back("src/proud_up/.env");
  paths.emplace_back(".env");
  return paths;
}

std::string find_dotenv_path() {
  for (const auto &path : dotenv_search_paths()) {
    if (file_exists(path)) {
      return path;
    }
  }
  return {};
}

int load_dotenv_file(const std::string &path) {
  std::ifstream in(path);
  if (!in) {
    return -1;
  }
  int applied = 0;
  std::string line;
  while (std::getline(in, line)) {
    EnvAssignment assignment;
    if (!parse_dotenv_line(line, assignment)) {
      continue;
    }
    setenv(assignment.key.c_str(), assignment.value.c_str(), 0);
    ++applied;
  }
  return applied;
}

std::string load_dotenv() {
  const auto path = find_dotenv_path();
  if (path.empty()) {
    return {};
  }
  if (load_dotenv_file(path) < 0) {
    return {};
  }
  return path;
}

}  // namespace proud_up
