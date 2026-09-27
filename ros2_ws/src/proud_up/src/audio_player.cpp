#include "proud_up/audio_player.hpp"

#include <chrono>
#include <csignal>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

// posix_spawnp reads the process environment (PATH, Pulse sink, …).
extern char **environ;

using namespace std::chrono_literals;

namespace proud_up {
namespace {

// Fork a child in its own process group so killpg() can stop ffplay AND
// any decoder it spawned. posix_spawnp (not std::system) so we keep the
// pid. argv pointers must stay valid until posix_spawnp returns — they
// alias the std::string c_str() of `args`, which is still in scope.
pid_t spawn_argv(const std::vector<std::string> &args) {
  if (args.empty()) {
    return -1;
  }
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto &a : args) {
    argv.push_back(const_cast<char *>(a.c_str()));
  }
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attr;
  posix_spawn_file_actions_init(&actions);
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attr, 0);
  pid_t pid = -1;
  const int rc = posix_spawnp(&pid, argv[0], &actions, &attr, argv.data(), environ);
  posix_spawnattr_destroy(&attr);
  posix_spawn_file_actions_destroy(&actions);
  if (rc != 0) {
    return -1;
  }
  return pid;
}

// SIGTERM, 80 ms grace, then SIGKILL. kill(..., 0) is a liveness probe
// (signal 0 never delivers); errno ESRCH means the child already exited.
void kill_group(pid_t pid) {
  if (pid <= 0) {
    return;
  }
  if (killpg(pid, SIGTERM) != 0) {
    kill(pid, SIGTERM);
  }
  std::this_thread::sleep_for(80ms);
  if (killpg(pid, 0) == 0 || kill(pid, 0) == 0) {
    killpg(pid, SIGKILL);
    kill(pid, SIGKILL);
  }
}

void wait_spawned(pid_t pid, double timeout_s) {
  if (pid <= 0) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    int status = 0;
    if (waitpid(pid, &status, WNOHANG) == pid) {
      return;
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (timeout_s > 0.0 && elapsed >= timeout_s) {
      kill_group(pid);
      waitpid(pid, &status, 0);
      return;
    }
    std::this_thread::sleep_for(20ms);
  }
}

void set_output_max() {
  wait_spawned(spawn_argv({"/usr/bin/pactl", "set-sink-mute", "@DEFAULT_SINK@", "0"}), 1.0);
  wait_spawned(
      spawn_argv({"/usr/bin/pactl", "set-sink-volume", "@DEFAULT_SINK@", "100%"}), 1.0);
}

bool looks_like_mp3(const std::string &path) {
  if (path.size() < 4) {
    return false;
  }
  const char *e = path.c_str() + path.size() - 4;
  return e[0] == '.' && (e[1] == 'm' || e[1] == 'M') && (e[2] == 'p' || e[2] == 'P') &&
         e[3] == '3';
}

}  // namespace

AudioPlayer::AudioPlayer() : AudioPlayer(AudioPlayerConfig{}) {}

AudioPlayer::AudioPlayer(AudioPlayerConfig config) : config_(std::move(config)) {}

AudioPlayer::~AudioPlayer() { stop(); }

bool AudioPlayer::clip_exists(const std::string &path) {
  struct stat st {};
  return !path.empty() && stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

void AudioPlayer::play_once(const std::string &path, double timeout_s) {
  stop();
  if (!clip_exists(path)) {
    finished_.store(true);
    return;
  }
  stop_.store(false);
  finished_.store(false);
  running_.store(true);
  path_ = path;
  timeout_s_ = timeout_s;
  th_ = std::thread([this]() { worker(); });
}

void AudioPlayer::stop() {
  stop_.store(true);
  const pid_t pid = pid_.load();
  if (pid > 0) {
    kill_group(pid);
  }
  if (th_.joinable()) {
    th_.join();
  }
  pid_.store(0);
  running_.store(false);
}

bool AudioPlayer::running() const { return running_.load(); }
bool AudioPlayer::finished() const { return finished_.load(); }

std::string AudioPlayer::wav_for_aplay() const {
  if (!looks_like_mp3(path_)) {
    return path_;
  }
  // 0.6 s of silence is what gets dropped if a later fallback still has
  // to use ffplay. aplay itself drains the device before it exits.
  const std::string &out = config_.wav_cache;
  const pid_t enc = spawn_argv({"/usr/bin/ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                                "-i", path_, "-af", "apad=pad_dur=0.6", "-ar", "44100", "-ac",
                                "2", out});
  if (enc <= 0) {
    return {};
  }
  wait_spawned(enc, 8.0);
  if (!clip_exists(out)) {
    return {};
  }
  return out;
}

void AudioPlayer::worker() {
  if (config_.boost_output) {
    set_output_max();
  }
  const std::string wav = wav_for_aplay();
  pid_t pid = -1;
  if (!wav.empty()) {
    pid = spawn_argv({"/usr/bin/aplay", "-q", "-D", "pulse", wav});
  }
  if (pid <= 0) {
    pid = spawn_argv({"/usr/bin/ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet",
                      "-volume", "100", "-af", "apad=pad_dur=0.8", path_});
  }
  if (pid <= 0) {
    running_.store(false);
    finished_.store(true);
    return;
  }
  pid_.store(pid);
  const auto start = std::chrono::steady_clock::now();
  while (!stop_.load()) {
    int status = 0;
    const pid_t r = waitpid(pid, &status, WNOHANG);
    if (r == pid) {
      break;
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (timeout_s_ > 0.0 && elapsed >= timeout_s_) {
      kill_group(pid);
      waitpid(pid, &status, 0);
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  pid_.store(0);
  running_.store(false);
  finished_.store(true);
}

}  // namespace proud_up
