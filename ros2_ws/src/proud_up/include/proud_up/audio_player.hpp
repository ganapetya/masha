#pragma once

#include <atomic>
#include <string>
#include <sys/types.h>
#include <thread>

namespace proud_up {

// One-shot clip player. A ROS timer calls play_once() and returns; the
// worker thread owns pactl, ffmpeg, and aplay. stop() kills that process
// group and joins the worker, so the node never waits on the speaker
// inside a callback.
//
// mp3 is transcoded to a padded wav (wav_cache) and played with
// `aplay -D pulse`, which drains the USB speaker before it exits.
// `ffplay -autoexit` closes Pulse at EOF and that speaker drops the
// queued tail — the clip starts, then the last syllable is missing.
// ffplay is only the fallback, and it pads too.
struct AudioPlayerConfig {
  // Scratch wav for the mp3 transcode. Two players that can overlap
  // need two paths; the default is the hunter NAME clip.
  std::string wav_cache{"/tmp/masha_hunter_name.wav"};
  // Unmute the default Pulse sink and set it to 100% before each clip.
  // The USB speaker boot script used to leave the sink at 80%, and the
  // first call after the sink suspends is quiet without this.
  bool boost_output{true};
};

class AudioPlayer {
 public:
  AudioPlayer();
  explicit AudioPlayer(AudioPlayerConfig config);
  ~AudioPlayer();

  AudioPlayer(const AudioPlayer &) = delete;
  AudioPlayer &operator=(const AudioPlayer &) = delete;
  AudioPlayer(AudioPlayer &&) = delete;
  AudioPlayer &operator=(AudioPlayer &&) = delete;

  // Missing file: finished() becomes true and no thread is started.
  void play_once(const std::string &path, double timeout_s);
  void stop();

  bool running() const;
  bool finished() const;

  // Regular file on disk. Playback and a node's startup log share this.
  static bool clip_exists(const std::string &path);

 private:
  void worker();
  std::string wav_for_aplay() const;

  AudioPlayerConfig config_;
  std::thread th_;
  std::string path_;
  double timeout_s_{3.0};
  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<pid_t> pid_{0};
};

}  // namespace proud_up
