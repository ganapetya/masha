// masha_interaction_node — one conversation, then a dance.
//
// Imagine a small theatre. Someone in the dark says "who is your master".
// Masha answers with a recorded line, then a piano-roll of servo pulses
// (the action group master_dance.d6a) turns her body while music fills
// the room. When the roll ends, the music stops and she waits again.
//
// This file is the stage manager. It does not hear the microphone
// (that is asr_node.py). It does not walk (that is voice_control_move.py).
// It does not move servos (that is MoveController + ActionGroupController).
// It only listens for the right sentence, then cues halt, speech, dance,
// and music in that order.
//
// The live path, as the code actually does it:
//
//   /asr_node/voice_words  "who is your master"
//        → Idle is listening. A match is logged as IDENTIFY, then we
//          enter Halt. IDENTIFY is a log line, not its own phase.
//        → Halt: zero /controller/cmd_vel, then Traveling gait=0,
//          then Traveling gait=-2 (DEFAULT_POSE, the real stand).
//          Wait halt_wait_s (0.4 s) so the stand can begin.
//        → Speak: play master_response.mp3 on a worker thread.
//          No spoken file → no dance. We go back to Idle.
//        → Dance: postcard on /controller/run_actionset
//          (action_path=master_dance, interrupt=true) and start music
//          in the same breath.
//        → Wait until /action_complete has gone false (the roll is
//          running) and then true (the roll finished) — or until
//          dance_timeout_s, or until someone says "stop".
//        → Stop the music, back to Idle.
//
// A ROS *topic* is a postcard: you publish and whoever is listening may
// pick it up. A *service* is a registered letter: you wait for a reply.
// /controller/run_actionset is a postcard (interfaces/msg/RunActionSet).
// There is no RunActionSet service on this robot. An older syllabus
// named yahboomcar_msgs::srv::RunActionSet; that package is not here.
//
// Why the completion postcard is /action_complete, not
// /controller/action_complete:
//   MoveController is a node named "controller". In ROS 2 a name with a
//   leading ~ (tilde) is private: ~/run_actionset becomes
//   /controller/run_actionset. A name *without* the tilde is only
//   relative to the namespace, which here is "/". So the publisher
//     create_publisher(Bool, 'action_complete', 1)
//   lands on /action_complete. That is the mailbox ActionGroupController
//   writes false (running) then true (finished) into. Live
//   `ros2 topic list` on Masha shows /action_complete and does not show
//   /controller/action_complete. We used to listen on the empty street.
//
// Threads — two rooms, one key, and a musician in the hall.
//   A MultiThreadedExecutor can run more than one callback at once.
//   We give speech and the 50 ms clock two separate "rooms"
//   (callback groups) that do not let two of their own callbacks in
//   together.
//     1. The speech room only writes a note (phase, abort, flags)
//        under mutex_ and leaves. Playing a sound here would freeze
//        the house the way aplay on the listen thread used to freeze
//        the microphone.
//     2. The clock room (tick) reads the note, then publishes halt
//        or the dance, and starts or stops the player.
//   The musician is a std::thread that posix_spawn's aplay (or ffplay).
//   We do not fork(). This process already has several threads; fork
//   would copy only the one that called it, and the others might be
//   holding locks. The copy would wait forever for a key that lives
//   in a room that was not copied.
//
// ASK / WAIT_YES / FOLLOW / ZMP are later Sprint 3 figures. The enum
// has a place to hang them; they are not wired here.

#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <mutex>
#include <spawn.h>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <interfaces/msg/run_action_set.hpp>
#include <kinematics_msgs/msg/traveling.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "proud_up/interaction_phrases.hpp"

// The environment pointer the OS already keeps for this process.
// posix_spawn needs it so aplay inherits PATH, Pulse, and the rest.
extern char **environ;

// Lets us write 50ms, 80ms instead of std::chrono::milliseconds(50).
using namespace std::chrono_literals;

namespace proud_up {
namespace {

// Start a helper program (ffmpeg, aplay, ffplay) as its own process.
// SETPGROUP puts it in a new process group so stop() can kill the
// whole family with one signal — ffplay sometimes has children.
pid_t spawn_argv(const std::vector<std::string> &args) {
  if (args.empty()) {
    return -1;
  }
  // posix_spawn wants a C array of char* ending in nullptr.
  // c_str() is the bytes inside each std::string; we do not copy them.
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
    errno = rc;
    return -1;
  }
  return pid;
}

void kill_group(pid_t pid) {
  if (pid <= 0) {
    return;
  }
  // Negative pid to killpg: "the group we made in posix_spawnattr_setpgroup".
  // Ask politely first (SIGTERM), then if the child is still there, SIGKILL.
  if (killpg(pid, SIGTERM) != 0) {
    kill(pid, SIGTERM);
  }
  std::this_thread::sleep_for(80ms);
  if (killpg(pid, 0) == 0 || kill(pid, 0) == 0) {
    killpg(pid, SIGKILL);
    kill(pid, SIGKILL);
  }
}

// Sit with a child until it exits, or we are told to stop, or time runs out.
// WNOHANG means "peek, do not sleep in waitpid" — we sleep 20 ms ourselves
// so we can notice stop_ in between peeks.
bool wait_pid_until(pid_t pid, std::atomic<bool> *stop, double timeout_s) {
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    int status = 0;
    const pid_t r = waitpid(pid, &status, WNOHANG);
    if (r == pid) {
      return WIFEXITED(status) && WEXITSTATUS(status) == 0;
    }
    if (stop && stop->load()) {
      kill_group(pid);
      waitpid(pid, &status, 0);
      return false;
    }
    if (timeout_s > 0.0) {
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      if (elapsed >= timeout_s) {
        kill_group(pid);
        waitpid(pid, &status, 0);
        return false;
      }
    }
    std::this_thread::sleep_for(20ms);
  }
}

bool file_exists(const std::string &path) {
  struct stat st {};
  return !path.empty() && stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

double file_mtime(const std::string &path) {
  struct stat st {};
  if (stat(path.c_str(), &st) != 0) {
    return 0.0;
  }
  return static_cast<double>(st.st_mtime);
}

std::string wav_cache_path(const std::string &mp3) {
  const auto dot = mp3.rfind('.');
  if (dot == std::string::npos) {
    return mp3 + ".wav";
  }
  return mp3.substr(0, dot) + ".wav";
}

// aplay is a simple piano: it plays wav, not mp3. ffmpeg transcribes the
// mp3 into a wav next to it the first time we need it, and again if the
// mp3 is newer. The wav is a cache (sounds/.gitignore). We do this on
// first play, inside the worker thread — not at node start, and not on
// the ROS callback.
bool ensure_wav_cache(const std::string &mp3, std::string *wav_out) {
  if (!file_exists(mp3)) {
    return false;
  }
  const std::string wav = wav_cache_path(mp3);
  if (file_exists(wav) && file_mtime(wav) >= file_mtime(mp3)) {
    *wav_out = wav;
    return true;
  }
  const pid_t pid = spawn_argv({
      "/usr/bin/ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
      "-i", mp3, "-ar", "44100", "-ac", "2", wav,
  });
  if (pid <= 0) {
    return false;
  }
  std::atomic<bool> never{false};
  if (!wait_pid_until(pid, &never, 20.0) || !file_exists(wav)) {
    return false;
  }
  *wav_out = wav;
  return true;
}

// The musician in the hall. voice_play.py kills aplay after 3 s — that
// clips the spoken line and cannot run a ~14 s dance track. This player
// takes an explicit timeout for speech, or none for music (the node
// kills the child when the dance ends).
//
// atomic flags (stop_, running_, finished_, pid_) are notes the ROS
// thread and this thread can both read without taking mutex_. A mutex
// is a key to a drawer; an atomic is a single light switch that the
// hardware lets two people flip safely.
class AudioPlayer {
 public:
  ~AudioPlayer() { stop(); }

  void play_once(const std::string &mp3, double timeout_s) {
    start(mp3, false, timeout_s);
  }

  void play_loop(const std::string &mp3) { start(mp3, true, 0.0); }

  void stop() {
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

  bool running() const { return running_.load(); }
  bool finished() const { return finished_.load(); }

 private:
  void start(const std::string &mp3, bool loop, double timeout_s) {
    stop();
    stop_.store(false);
    finished_.store(false);
    running_.store(true);
    mp3_ = mp3;
    loop_ = loop;
    timeout_s_ = timeout_s;
    th_ = std::thread([this]() { worker(); });
  }

  void worker() {
    std::string wav;
    const bool have_wav = ensure_wav_cache(mp3_, &wav);
    auto spawn_play = [&]() -> pid_t {
      if (have_wav) {
        // pulse only. After reboot, a raw USB device name can stall in
        // the kernel. voice_play.py also tries "default" and a plughw
        // card; this player does not. If pulse fails, we fall through
        // to ffplay on the mp3 itself.
        return spawn_argv({"/usr/bin/aplay", "-q", "-D", "pulse", wav});
      }
      return spawn_argv({"/usr/bin/ffplay", "-nodisp", "-autoexit",
                         "-loglevel", "quiet", mp3_});
    };

    bool ok = false;
    do {
      if (stop_.load()) {
        break;
      }
      pid_t pid = spawn_play();
      if (pid <= 0 && have_wav) {
        pid = spawn_argv({"/usr/bin/ffplay", "-nodisp", "-autoexit",
                          "-loglevel", "quiet", mp3_});
      }
      if (pid <= 0) {
        break;
      }
      pid_.store(pid);
      ok = wait_pid_until(pid, &stop_, timeout_s_);
      pid_.store(0);
      if (!ok && have_wav && !loop_ && !stop_.load()) {
        pid = spawn_argv({"/usr/bin/ffplay", "-nodisp", "-autoexit", "-loglevel",
                          "quiet", mp3_});
        if (pid > 0) {
          pid_.store(pid);
          ok = wait_pid_until(pid, &stop_, timeout_s_);
          pid_.store(0);
        }
      }
      if (!ok && !loop_) {
        break;
      }
      if (loop_ && !stop_.load()) {
        std::this_thread::sleep_for(30ms);
      }
    } while (loop_ && !stop_.load());

    running_.store(false);
    finished_.store(true);
    (void)ok;
  }

  std::thread th_;
  std::atomic<bool> stop_{true};
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{true};
  std::atomic<pid_t> pid_{0};
  std::string mp3_;
  bool loop_{false};
  double timeout_s_{0.0};
};

}  // namespace

// enum class: a typed list of names. Idle is not the number 0 in
// disguise; you cannot accidentally add it to an int. The four
// values below are the only figures this slice performs.
// LISTEN in the memo is Idle. IDENTIFY is a log inside on_speech,
// then we walk straight into Halt. There is no separate IDENTIFY
// state, and Idle after the dance is the same Idle as before it.
enum class Phase {
  Idle,  // waiting for the master question, or for "stop"
  Halt,
  Speak,
  Dance,
  // ASK, WAIT_YES, FOLLOW — Sprint 3 Step 2, not this slice.
};

const char *phase_cstr(Phase p) {
  switch (p) {
    case Phase::Idle:
      return "idle";
    case Phase::Halt:
      return "halt";
    case Phase::Speak:
      return "speak";
    case Phase::Dance:
      return "dance";
  }
  return "unknown";
}

class MashaInteractionNode : public rclcpp::Node {
 public:
  MashaInteractionNode() : Node("masha_interaction_node") {
    // declare_parameter: if the launch YAML names the key, use that;
    // otherwise use the default written here. Both should agree.
    asr_topic_ = declare_parameter<std::string>("asr_topic", "/asr_node/voice_words");
    traveling_topic_ =
        declare_parameter<std::string>("traveling_topic", "/controller/traveling");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/controller/cmd_vel");
    run_actionset_topic_ =
        declare_parameter<std::string>("run_actionset_topic", "/controller/run_actionset");
    action_complete_topic_ = declare_parameter<std::string>(
        "action_complete_topic", "/action_complete");
    action_name_ = declare_parameter<std::string>("action_name", "master_dance");
    master_response_mp3_ = declare_parameter<std::string>(
        "master_response_mp3",
        "/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english/"
        "master_response.mp3");
    music_mp3_ = declare_parameter<std::string>(
        "music_mp3", "/home/ubuntu/ros2_ws/src/proud_up/sounds/master_dance.mp3");
    music_loop_ = declare_parameter<bool>("music_loop", true);
    halt_wait_s_ = declare_parameter<double>("halt_wait_s", 0.4);
    speak_timeout_s_ = declare_parameter<double>("speak_timeout_s", 8.0);
    // master_dance.d6a is 25 rows, 14.3 s. 16.5 s is a backstop, not
    // a sleep that "is" the dance.
    dance_timeout_s_ = declare_parameter<double>("dance_timeout_s", 16.5);

    speech_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    traveling_pub_ =
        create_publisher<kinematics_msgs::msg::Traveling>(traveling_topic_, 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
    run_actionset_pub_ =
        create_publisher<interfaces::msg::RunActionSet>(run_actionset_topic_, 1);

    // [this] on a lambda: when the postcard arrives, call a method on
    // *this* node. ROS keeps the subscription alive because we store
    // the SharedPtr (asr_sub_, complete_sub_) as a member.
    rclcpp::SubscriptionOptions speech_opts;
    speech_opts.callback_group = speech_cb_group_;
    asr_sub_ = create_subscription<std_msgs::msg::String>(
        asr_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_speech(msg); },
        speech_opts);
    complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        action_complete_topic_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { on_action_complete(msg); },
        speech_opts);

    // Same "are you up?" doorbell the other proud_up nodes offer.
    // ~/init_finish becomes /masha_interaction_node/init_finish.
    // The reply's message field is the current phase name.
    init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/init_finish",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          response->success = true;
          response->message = phase_cstr(phase_);
        });

    timer_ = create_wall_timer(50ms, [this]() { tick(); }, timer_cb_group_);
    phase_started_ = now();

    RCLCPP_INFO(get_logger(),
                "masha_interaction asr=%s action=%s complete=%s "
                "reply_mp3=%s music_mp3=%s (mp3 missing: silent dance, no reply → no dance)",
                asr_topic_.c_str(), action_name_.c_str(), action_complete_topic_.c_str(),
                master_response_mp3_.c_str(), music_mp3_.c_str());
    if (!file_exists(master_response_mp3_)) {
      RCLCPP_WARN(get_logger(),
                  "spoken mp3 not on disk yet: %s — dance will be refused until it is",
                  master_response_mp3_.c_str());
    }
    if (!file_exists(music_mp3_)) {
      RCLCPP_INFO(get_logger(),
                  "dance music mp3 not on disk yet: %s — dance still runs silent",
                  music_mp3_.c_str());
    }
  }

  ~MashaInteractionNode() override { shutdown_player(); }

  void shutdown_player() { player_.stop(); }

 private:
  rclcpp::Time now() { return get_clock()->now(); }

  double elapsed_s() {
    return (now() - phase_started_).seconds();
  }

  void on_speech(const std_msgs::msg::String::SharedPtr msg) {
    // Store and return. Matching a string is cheap; aplay is not done here.
    // lock_guard is RAII: the key (the mutex) is taken when lock is built
    // and given back when we leave this function, even on an early return.
    const std::string &text = msg->data;
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_stop_command(text) && (phase_ == Phase::Speak || phase_ == Phase::Dance ||
                                  phase_ == Phase::Halt)) {
      abort_ = true;
      RCLCPP_INFO(get_logger(), "stop during %s — abort dance / speak", phase_cstr(phase_));
      return;
    }
    if (phase_ != Phase::Idle) {
      return;
    }
    if (!is_master_query(text)) {
      return;
    }
    // IDENTIFY lives here, not as a Phase. ASR usually already published
    // the canonical "who is your master"; aliases still match if a raw
    // transcript ever arrives. Then we step into Halt.
    RCLCPP_INFO(get_logger(), "IDENTIFY %s (raw=%s)", kMasterQueryCanonical, text.c_str());
    phase_ = Phase::Halt;
    halt_sent_ = false;
    abort_ = false;
    dance_sent_ = false;
    saw_action_running_ = false;
    phase_started_ = now();
  }

  void on_action_complete(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    action_complete_ = msg->data;
    if (!msg->data) {
      // false = the .d6a thread is running. We must see this before we
      // treat a later true as "this dance finished". Otherwise a leftover
      // true from the previous group would end Dance on the first tick —
      // clapping for a bow that already happened.
      saw_action_running_ = true;
    }
  }

  void tick() {
    // Copy the note, drop the key, then act. Publishing while holding
    // mutex_ would invite a deadlock if a callback needed the same key.
    Phase phase;
    bool abort;
    bool halt_sent;
    bool dance_sent;
    bool saw_running;
    bool action_complete;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase = phase_;
      abort = abort_;
      halt_sent = halt_sent_;
      dance_sent = dance_sent_;
      saw_running = saw_action_running_;
      action_complete = action_complete_;
    }

    switch (phase) {
      case Phase::Idle:
        break;
      case Phase::Halt:
        tick_halt(halt_sent, abort);
        break;
      case Phase::Speak:
        tick_speak(abort);
        break;
      case Phase::Dance:
        tick_dance(dance_sent, saw_running, action_complete, abort);
        break;
    }
  }

  void tick_halt(bool halt_sent, bool abort) {
    if (abort) {
      enter_idle("abort before speak");
      return;
    }
    if (!halt_sent) {
      halt_legs();
      std::lock_guard<std::mutex> lock(mutex_);
      halt_sent_ = true;
      phase_started_ = now();
      return;
    }
    if (elapsed_s() < halt_wait_s_) {
      return;
    }
    if (!file_exists(master_response_mp3_)) {
      RCLCPP_ERROR(get_logger(),
                   "no spoken mp3 at %s — not dancing without the line",
                   master_response_mp3_.c_str());
      enter_idle("missing master_response.mp3");
      return;
    }
    RCLCPP_INFO(get_logger(), "SPEAK %s", master_response_mp3_.c_str());
    player_.play_once(master_response_mp3_, speak_timeout_s_);
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Speak;
    phase_started_ = now();
  }

  void tick_speak(bool abort) {
    if (abort) {
      player_.stop();
      halt_legs();
      enter_idle("abort during speak");
      return;
    }
    const bool timed_out = elapsed_s() >= speak_timeout_s_;
    if (!player_.finished() && !timed_out) {
      return;
    }
    if (timed_out && player_.running()) {
      RCLCPP_WARN(get_logger(), "speak timed out at %.1f s — cutting aplay, still dancing",
                  speak_timeout_s_);
      player_.stop();
    }
    start_dance();
  }

  void start_dance() {
    // Postcard, not a letter. action_path is the basename of the .d6a
    // (no ".d6a", no folder). Do not call this dance_1 / dance_2 /
    // dance_3 / stop — MoveController treats those as other plays.
    interfaces::msg::RunActionSet msg;
    msg.action_path = action_name_;
    msg.interrupt = true;
    run_actionset_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "DANCE RunActionSet action_path=%s (topic, not a service)",
                action_name_.c_str());

    if (file_exists(music_mp3_)) {
      if (music_loop_) {
        player_.play_loop(music_mp3_);
      } else {
        player_.play_once(music_mp3_, dance_timeout_s_);
      }
    } else {
      RCLCPP_WARN(get_logger(), "no dance music at %s — dancing silent", music_mp3_.c_str());
    }

    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Dance;
    dance_sent_ = true;
    saw_action_running_ = false;
    action_complete_ = false;
    phase_started_ = now();
  }

  void tick_dance(bool /*dance_sent*/, bool saw_running, bool action_complete, bool abort) {
    const bool done = (saw_running && action_complete) || elapsed_s() >= dance_timeout_s_;
    if (!abort && !done) {
      return;
    }
    player_.stop();
    if (abort) {
      // action_path "stop" is a special word in MoveController:
      // halt the group, then play init_pose. Speak-abort does not
      // send this (the dance has not started). Halt-abort only
      // returns to Idle.
      interfaces::msg::RunActionSet msg;
      msg.action_path = "stop";
      msg.interrupt = true;
      run_actionset_pub_->publish(msg);
      halt_legs();
      enter_idle("abort during dance");
      return;
    }
    if (elapsed_s() >= dance_timeout_s_ && !(saw_running && action_complete)) {
      RCLCPP_WARN(get_logger(),
                  "dance timeout %.1f s (no /action_complete true) — stopping music",
                  dance_timeout_s_);
    } else {
      RCLCPP_INFO(get_logger(), "dance complete — music stopped");
    }
    enter_idle("dance finished");
  }

  void enter_idle(const char *why) {
    RCLCPP_INFO(get_logger(), "IDLE (%s)", why);
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Idle;
    abort_ = false;
    halt_sent_ = false;
    dance_sent_ = false;
    phase_started_ = now();
  }

  // Standing still is not the same as walking at speed zero.
  // /controller/cmd_vel always starts a gait generator, even when every
  // number in the Twist is 0 — she would march in place. follow_the_cat
  // therefore never sends a lone zero Twist. We still send one first, so
  // a leftover walk from voice_control_move does not keep that generator
  // alive, then we send the real halt:
  //   gait 0  = stop the stepping loop
  //   gait -2 = snap to DEFAULT_POSE (the stand voice already uses)
  // gait=-2 last, so the stand wins if both postcards arrive in one tick.
  // Traveling.msg also has steps, stride, and friends; we leave them at
  // their zeros. interrupt=true means "drop what you were doing".
  void halt_legs() {
    geometry_msgs::msg::Twist zero;
    cmd_vel_pub_->publish(zero);

    kinematics_msgs::msg::Traveling stop;
    stop.gait = 0;
    stop.time = 1.0f;
    stop.interrupt = true;
    traveling_pub_->publish(stop);

    kinematics_msgs::msg::Traveling stand;
    stand.gait = -2;
    stand.time = 1.0f;
    stand.interrupt = true;
    traveling_pub_->publish(stand);
    RCLCPP_INFO(get_logger(), "halt Traveling gait=0 then gait=-2 (DEFAULT_POSE)");
  }

  std::string asr_topic_;
  std::string traveling_topic_;
  std::string cmd_vel_topic_;
  std::string run_actionset_topic_;
  std::string action_complete_topic_;
  std::string action_name_;
  std::string master_response_mp3_;
  std::string music_mp3_;
  bool music_loop_{true};
  double halt_wait_s_{0.4};
  double speak_timeout_s_{8.0};
  double dance_timeout_s_{16.5};

  rclcpp::CallbackGroup::SharedPtr speech_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::Publisher<kinematics_msgs::msg::Traveling>::SharedPtr traveling_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<interfaces::msg::RunActionSet>::SharedPtr run_actionset_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr asr_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  Phase phase_{Phase::Idle};
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};
  bool halt_sent_{false};
  bool dance_sent_{false};
  bool abort_{false};
  bool saw_action_running_{false};
  bool action_complete_{true};

  AudioPlayer player_;
};

}  // namespace proud_up

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<proud_up::MashaInteractionNode>();
  // weak_ptr: a polite glance at the node without keeping it alive.
  // If shutdown races with destruction, lock() returns empty and we
  // do not call a dead object.
  std::weak_ptr<proud_up::MashaInteractionNode> weak = node;
  rclcpp::on_shutdown([weak]() {
    if (auto n = weak.lock()) {
      n->shutdown_player();
    }
  });

  // MultiThreadedExecutor: more than one callback may run at once,
  // which is why the two rooms (callback groups) and the mutex exist.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  node->shutdown_player();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
