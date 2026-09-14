#pragma once

// Phrase matching for masha_interaction_node. No rclcpp — gtest includes
// this header and checks the aliases without spinning a node.
//
// ASR (asr_node.py extract_command) already maps many spellings onto the
// canonical string "who is your master" and publishes that on
// /asr_node/voice_words. This matcher still accepts the raw-ish forms in
// case a future path publishes the transcript instead of the canonical
// command. Keep the alias list in sync with COMMAND_PHRASES in asr_node.py.

#include <cctype>
#include <string>
#include <string_view>

namespace proud_up {

inline constexpr const char *kMasterQueryCanonical = "who is your master";
inline constexpr const char *kStopCanonical = "stop";

// Same normalisation as asr_node.extract_command: lower case, every
// non-alphanumeric byte becomes a space, runs of spaces collapse. Apostrophes
// disappear, so "who's" → "who s". That is why kMasterAliases includes
// "who s your master".
inline std::string normalize_speech(std::string_view raw) {
  std::string out;
  out.reserve(raw.size());
  bool pending_space = false;
  for (unsigned char c : raw) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<unsigned char>(c - 'A' + 'a');
    }
    const bool word = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c >= 0x80);
    if (word) {
      if (pending_space && !out.empty()) {
        out.push_back(' ');
      }
      out.push_back(static_cast<char>(c));
      pending_space = false;
    } else {
      pending_space = true;
    }
  }
  return out;
}

inline bool is_master_query(std::string_view raw) {
  const std::string n = normalize_speech(raw);
  if (n.empty()) {
    return false;
  }
  // Substring, not token-equality: "masha who is your master" must match.
  // Longer / more specific aliases are listed first only for readability;
  // find() does not care about order as long as no alias is a substring of
  // an unrelated command (none of these appear in "go forward" / "dance").
  static constexpr const char *kMasterAliases[] = {
      "who is your master", "who is ur master", "whose your master",
      "who s your master",  "who your master",
  };
  for (const char *alias : kMasterAliases) {
    if (n.find(alias) != std::string::npos) {
      return true;
    }
  }
  return false;
}

inline bool is_stop_command(std::string_view raw) {
  const std::string n = normalize_speech(raw);
  if (n.empty()) {
    return false;
  }
  if (n == kStopCanonical) {
    return true;
  }
  // Word-boundary "stop" so "unstoppable" would not fire. ASR publishes the
  // canonical "stop"; this only helps if a raw transcript leaks through.
  if (n.size() >= 4 && n.compare(0, 5, "stop ") == 0) {
    return true;
  }
  if (n.size() >= 5 && n.compare(n.size() - 5, 5, " stop") == 0) {
    return true;
  }
  return n.find(" stop ") != std::string::npos;
}

}  // namespace proud_up
