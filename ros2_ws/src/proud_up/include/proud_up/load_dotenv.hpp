#pragma once

#include <string>
#include <vector>

namespace proud_up {

struct EnvAssignment {
  std::string key;
  std::string value;
};

// Parse one dotenv line. Returns false for blanks, comments, and malformed lines.
bool parse_dotenv_line(const std::string &line, EnvAssignment &out);

// Candidate paths: $PROUD_UP_ENV_FILE, package source .env, cwd .env.
std::vector<std::string> dotenv_search_paths();

// First existing file from dotenv_search_paths(), or empty.
std::string find_dotenv_path();

// Load KEY=VALUE into the process environment. Existing vars are left alone.
// Returns how many keys were set from that file, or -1 if the file cannot be read.
int load_dotenv_file(const std::string &path);

// Find and load. Returns the path loaded, or empty if none.
std::string load_dotenv();

}  // namespace proud_up
