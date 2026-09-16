# How posing works on Masha

This page is a teacher’s walk-through of **one stand and one dance** on this robot. Voice is out. Treat the C++ node as already having decided “now we pose.” How she heard the sentence is in [`MASTER_DIALOG_DANCE.md`](MASTER_DIALOG_DANCE.md). How a `.d6a` file is recorded is in [`DANCE_ACTION_SETS.md`](DANCE_ACTION_SETS.md). How walking works is in [`MASHA_GAITS.md`](MASHA_GAITS.md).

Worked example: the pose sequence cued by `ros2_ws/src/proud_up/src/masha_interaction_node.cpp`.

**How to read this page.** First you meet the people. Then you read the story in time order. Boxes of arrows are only summaries. Under every box, the next paragraphs explain each label: **who sent it, what the words mean, who received it, what that receiver did.** If a word is new, it is defined the first time it appears.

Copies: `~/steps/DANCE_ACTION_SETS_DETAILS.md`

---

## 1. The people

Software on Masha is split into **processes** (OS programs) and **nodes** (ROS 2 objects inside those programs). A node can publish a **topic** (a named mailbox anyone may write to or read). Think of a topic as a postcard: the sender does not wait for a reply.

Here are the people who touch posing. You do not need to memorise file paths yet; they come back with the story.

### 1.1 The director — `masha_interaction_node`

A C++ ROS node in package `proud_up`. File: `ros2_ws/src/proud_up/src/masha_interaction_node.cpp`.

This node **does not move servos**. It does not know inverse kinematics. It does not open the dance file. It only decides *when* the body should stand still and *when* the dance file should start, then it watches for “the dance finished.”

It has a **50 millisecond timer**. Twenty times a second it runs a function named `tick()`. That timer is the director’s clock. It is not the servo clock.

It speaks to the body with three postcards:

| Postcard (topic) | Type | Plain meaning |
|---|---|---|
| `/controller/cmd_vel` | `geometry_msgs/Twist` | “here is a walking speed” (even zeros still mean a speed command) |
| `/controller/traveling` | `kinematics_msgs/Traveling` | “here is a gait / stand / stop, named by a number called `gait`” |
| `/controller/run_actionset` | `interfaces/msg/RunActionSet` | “play this named `.d6a` file” |

It listens on `/action_complete` (`std_msgs/Bool`) for “the file started” (`false`) and “the file finished” (`true`).

### 1.2 The receptionist — node `/controller` (`MoveController`)

A Python ROS node. File: `ros2_ws/src/driver/controller/controller/move_controller.py`. Process name is `move_controller`; the node’s ROS name is `controller`, so private topics with a `~` become `/controller/...`.

This node is the **public front door**. Almost every body command on the robot arrives here. It does not run the 20 ms inverse-kinematics loop itself. It translates incoming messages into calls on the worker sitting next to it (`StepController`), or it hands a dance file to the pianist (`ActionGroupController`).

### 1.3 The 20 ms worker — node `/step_controller` (`StepController`)

Same process as the receptionist. File: `ros2_ws/src/driver/controller/controller/step_controller.py`.

This node owns a **background thread** that wakes every 20 milliseconds. That thread is the IK posing clock. When someone has queued a stand, a body lean, or a walk, this thread computes where the six feet should be, turns that into joint angles, and publishes servo pulses.

When nobody has queued anything, the thread still wakes, sees an empty inbox, and goes back to sleep. That idle ticking does **not** move the legs.

### 1.4 The pianist — `ActionGroupController`

A Python class, not its own ROS node. File: `ros2_ws/src/driver/servo_controller/servo_controller/action_group_controller.py`. The receptionist constructs it and keeps it.

The pianist opens a SQLite file (`*.d6a`), reads rows, and publishes raw servo pulses. It does not think about feet or balance. It is a cursor over a piano roll.

**It does not know coordinates.** A pulse is not an xyz of a foot. It is the motor’s own position number (0–1000) for one physical servo. Those numbers were **recorded earlier**, usually in the Qt ActionSet editor, by reading the servos after a human posed the legs, or by dragging sliders. The pianist only **plays the recording back**. How that recording is made, and why no IK is needed, is §9.

### 1.5 The translator — `JointControl`

Python class/node used by the 20 ms worker. File: `ros2_ws/src/driver/kinematics/kinematics/x_joint_control.py`.

Inverse kinematics returns **radians**. Bus servos want **pulses** 0–1000. This class does that conversion and publishes on `/servo_controller`.

### 1.6 The last software before the wire — `/controller_manager`

Python node in package `servo_controller`. File: `ros2_ws/src/driver/servo_controller/servo_controller/controller_manager.py`.

It listens on `/servo_controller`. When a pulse message arrives, it forwards it toward the board. It also publishes `/joint_states` every 20 ms for RViz. That second 20 ms is **display**, not posing.

### 1.7 The serial talker — `/ros_robot_controller`

Python node. File: `ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py`.

It turns a ROS servo message into a USB packet on `/dev/ttyACM0`.

### 1.8 The interpolator — STM32 on the servo board

Firmware on the robot’s controller board. It receives “go to these pulses in this many milliseconds” and **slides** each servo from wherever it is now to the new number. Python does not draw that slide. If a new target arrives before the old slide is finished, the board starts a new slide from the current position.

---

## 2. Three ways the body can “pose”

Masha has no single pose API. Three programs share the last stretch of cable to the STM32. They must not run at once, or they fight over `/servo_controller`.

```
(1) Named IK stand     e.g. DEFAULT_POSE
(2) Body lean          PoseTransformer  (feet planted, body tilts)
(3) Action group       .d6a piano roll  (raw pulses)
```

**What those three names mean.**

**(1) Named IK stand.** A programmer stored six foot-tip positions in millimetres (`DEFAULT_POSE` in `build_in_pose.py`). Inverse kinematics (IK) means: given a foot’s xyz, compute the three joint angles of that leg. The 20 ms worker does that for all six legs **once**, publishes one servo message whose `duration` is 1.0 second, and the STM32 blends there. The dance uses this as the halt/stand.

**(2) Body lean.** Same six feet stay on the floor. The body is translated and rotated a little every 20 ms. That is what the Python files call “pose transformation.” The dance **does not** use this. It is later in this page so the name is not a mystery.

**(3) Action group.** A file of keyframes recorded in the Qt editor. Each row is “servos 1–24, these pulses, in this many milliseconds.” No IK. The dance’s choreography is this.

The dance we built is **(1) then (3)**: stand with IK, then play `master_dance.d6a`.

---

## 3. The whole path, as a map

This box is only a map of who talks to whom. Meanings are under it.

```
director   masha_interaction_node     (50 ms tick; does not move servos)
              |
              |  three postcards, explained in §5 and §8
              v
receptionist   /controller            MoveController
              |                    \
              | stand / lean / walk  \ dance file name
              v                       v
worker         /step_controller       pianist  ActionGroupController
               20 ms IK thread        background thread, sleeps per row
              |                       |
              +-----------+-----------+
                          v
topic          /servo_controller      (pulses + a duration)
                          v
               /controller_manager    (clamp duration, forward)
                          v
               /ros_robot_controller  (USB packet)
                          v
               STM32                  (slide pulses over duration)
                          v
               bus servos 1–24
```

**Who is on the left column.** The director is the C++ node. It only sends postcards. The receptionist is `MoveController`. If the postcard is “stand” or “walk” or “lean,” the receptionist writes into the worker’s inbox; the worker’s 20 ms thread is what actually computes legs. If the postcard is “play `master_dance`,” the receptionist does **not** use the worker. It asks the pianist, a separate thread that reads a file.

**Who is on the bottom.** Both the worker and the pianist eventually publish the same topic, `/servo_controller`. From there one path remains: manager → serial node → STM32 → motors. That is why a leftover walk and a dance cannot overlap: they would both write that topic.

**What “duration” is.** Every servo message carries a time. It is not a sleep in the STM32’s sense; it is “arrive at these pulses in this many seconds.” The pianist also `sleep()`s that time in Python so it does not send the next row early. The IK stand usually sends duration 1.0 s and does **not** sleep 1.0 s in the 20 ms thread — it publishes once and continues looping.

---

## 4. How the director is started (still no voice)

Launch file: `ros2_ws/src/proud_up/launch/masha_interaction.launch.py`.  
Parameters: `ros2_ws/src/proud_up/config/masha_interaction.yaml`.

On construction the director creates the three publishers and the 50 ms timer:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:404-436
traveling_pub_ =
    create_publisher<kinematics_msgs::msg::Traveling>(traveling_topic_, 1);
cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
run_actionset_pub_ =
    create_publisher<interfaces::msg::RunActionSet>(run_actionset_topic_, 1);
// ...
timer_ = create_wall_timer(50ms, [this]() { tick(); }, timer_cb_group_);
```

**What that code is doing.** `create_publisher` means “I will write this mailbox.” The mailbox names come from the YAML (`/controller/traveling`, `/controller/cmd_vel`, `/controller/run_actionset`). `create_wall_timer(50ms, tick)` means “call `tick()` every 50 ms on a wall clock.” `tick` is the director’s only regular work.

The director’s private state is a small enum:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:352-356
enum class Phase { Idle, Halt, Speak, Dance };
```

**What the four names mean for posing** (Speak exists only as a wait; we do not describe audio):

| Phase | Plain meaning |
|---|---|
| `Idle` | Do nothing to the body. |
| `Halt` | This is where posing **starts**: send the stand, then wait 0.4 s. |
| `Speak` | Wait until the spoken line is done (or times out). No new body command. |
| `Dance` | The dance file has been asked for. Wait until it finishes. |

Every 50 ms, `tick()` looks at `phase_` and calls one of three functions:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:509-540
void tick() {
  // copy flags under mutex, drop the lock, then:
  switch (phase) {
    case Phase::Idle:  break;
    case Phase::Halt:  tick_halt(...); break;
    case Phase::Speak: tick_speak(...); break;
    case Phase::Dance: tick_dance(...); break;
  }
}
```

**Who puts the node into `Halt`.** For this page: something already did (in the real robot, a speech callback). From `Halt` onward we only care about the body.

---

## 5. Halt — who commands it, what is in the postcards, who receives them

The first time `tick()` sees `Halt` and has not yet sent the stand, it calls `halt_legs()`:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:543-553
void tick_halt(bool halt_sent, bool abort) {
  if (abort) { enter_idle("abort before speak"); return; }
  if (!halt_sent) {
    halt_legs();          // send the three postcards, once
    halt_sent_ = true;
    phase_started_ = now();
    return;
  }
  if (elapsed_s() < halt_wait_s_) { return; }  // 0.4 s pause
  // then Speak (skipped here), then start_dance()
}
```

**Who commanded halt.** The director, function `halt_legs`, on the first Halt tick. Nobody else.

**Halt is not a ROS type.** There is no message named Halt. Halt is the director’s *phase*. The *content* of halt is three ordinary messages:

```
director
   |  1. Twist with every field 0          →  /controller/cmd_vel
   |  2. Traveling { gait: 0,  time: 1.0 } →  /controller/traveling
   |  3. Traveling { gait: -2, time: 1.0 } →  /controller/traveling
   v
receptionist /controller
```

Now each postcard, slowly.

### 5.1 Postcard 1 — “Twist 0”

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:671-673
geometry_msgs::msg::Twist zero;
cmd_vel_pub_->publish(zero);
```

**What a Twist is.** A walking-speed command: linear x/y (metres per second) and angular z (radians per second). The director fills nothing, so every number is 0.

**Is Twist the halt result?** No. It is **content going out**, not a result coming back. The director hopes a leftover *non-zero* walk will be replaced. The stand is still two postcards away.

**Who receives it.** `MoveController.cmd_vel_callback` (`move_controller.py:194-201`), which always calls `StepController.cmd_vel`. That **starts a gait generator even at speed 0**. Alone, this postcard would make her march in place. That is why it is never left alone.

### 5.2 Postcard 2 — “Traveling gait = 0”

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:675-679
kinematics_msgs::msg::Traveling stop;
stop.gait = 0;
stop.time = 1.0f;
stop.interrupt = true;
traveling_pub_->publish(stop);
```

**What Traveling is.** A message with a field `gait` (an integer that selects a behaviour), plus stride, height, and `time` (seconds, reused as “how long this should take”). File: `ros2_ws/src/driver/kinematics_msgs/msg/Traveling.msg`.

**What `gait = 0` means on this robot.** Not “walk with gait number zero.” In `set_traveling_callback` it means **stop the stepping loop**.

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:177-183
if msg.gait == 0:
    self.step_controller.stop_running(
        timeout=None,
        callback=lambda: self.step_controller.set_pose(None, None, msg.time)
    )
```

**Who receives it.** The receptionist. `stop_running(timeout=None)` **blocks that callback** until the 20 ms worker has no walk/lean/pose-in-flight (`stopped` Event). Then the lambda queues “hold the current six-foot pose for `msg.time` (1.0 s).” Holding current pose is not yet the pretty stand; it is “freeze whatever the feet are doing.”

**What `interrupt = true` means in the comment on the C++ side.** “Drop what you were doing.” The Python `gait == 0` path does that by clearing the worker’s queued generators. The `interrupt` field is not read on the `gait == -2` path.

### 5.3 Postcard 3 — “Traveling gait = -2”

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:681-686
kinematics_msgs::msg::Traveling stand;
stand.gait = -2;
stand.time = 1.0f;
stand.interrupt = true;
traveling_pub_->publish(stand);
```

**What `gait = -2` means.** “Go to the named pose `DEFAULT_POSE`.” Negative gaits are not walks. `-1` is `SLAM_POSE`. `-2` is the normal stand.

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:186-189
elif msg.gait == -2:
    self.step_controller.set_build_in_pose('DEFAULT_POSE', msg.time)
```

**Who receives it.** Again the receptionist, same callback, a **second** Traveling message (ROS delivers the two Traveling postcards separately). `set_build_in_pose` does not talk to servos on this thread. It only writes the worker’s inbox:

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:366-377
def set_build_in_pose(self, pose_name, duration, interrupt=True):
    with self.lock:
        pose = getattr(build_in_pose, pose_name)
        transform = getattr(build_in_pose, pose_name + '_TRANSFORM')
        self.new_pose_setter = (pose, transform, duration)
```

**What `DEFAULT_POSE` is.** Six foot tips, millimetres, body frame (+X toward the head, +Y to the robot’s left, +Z up, foot z negative). About 70 mm stance height. Defined in `ros2_ws/src/driver/controller/controller/build_in_pose.py:15-20`. `DEFAULT_POSE_TRANSFORM` at line 62 is the body offset the worker stores, `(0, 0, 130) mm`, rotation zero.

**Why three postcards, this order.** (1) replace a leftover walk speed so that generator is not “the current one.” (2) wait until the 20 ms loop is idle. (3) queue the real stand last, so if both Traveling messages are processed close together, `DEFAULT_POSE` overwrites the “hold current” from gait 0. The stand wins.

**The 0.4 s wait.** After sending, the director does nothing for `halt_wait_s` (0.4 s). The stand’s servo `duration` is 1.0 s. So Speak, and then the dance, **may start while the stand is still sliding**. That is why the first row of `master_dance.d6a` should itself look like a stand.

---

## 6. The 20 ms tick — what the worker does with the stand

The receptionist and the worker share one process:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:276-282
node = MoveController('controller')
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.add_node(node.step_controller)
executor.spin()
```

**What that means.** ROS will run both nodes’ callbacks. The 20 ms loop is **not** a ROS timer. It is a daemon thread started in `StepController.__init__`:

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:85-87
self.loop_thread = threading.Thread(target=self.loop, daemon=True)
self.loop_enable = True
self.loop_thread.start()
```

One pass of `loop()` is one tick. At the bottom: `time.sleep(0.02)`. Intended rate: 50 Hz.

Each tick always tries four jobs, **in this order**. For halt, only job A does work.

```
20 ms tick
  A. If a named pose is queued          → IK once, publish, clear everything else
  B. If a body-lean generator is queued → one lean frame
  C. If a walk generator is queued      → one gait frame
  D. If A/B/C produced a new 6-foot pose object → publish it
  then sleep 0.02 s
```

### 6.1 Job A — the stand is applied here

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:138-150
if self.new_pose_setter is not None:
    self.cur_pose_setter = self.new_pose_setter
    self.new_pose_setter = None
if self.cur_pose_setter is not None:
    pose, transform, duration = self.cur_pose_setter
    if pose is None or transform is None:
        self.set_pose_base(self.pose, duration, update_pose=True)  # hold current
    else:
        self.set_pose_base(pose, duration, update_pose=True)       # DEFAULT_POSE, 1.0 s
        self.transform = transform
    self.reset_all_cur_gen()
    self.reset_all_new_gen()
```

**Who queued this.** The receptionist, when it handled `gait == -2`. The worker finds `new_pose_setter = (DEFAULT_POSE, TRANSFORM, 1.0)`.

**What `set_pose_base` does.** This is the only IK-to-servo function the loop uses.

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:331-350
def set_pose_base(self, new_pose, duration, pseudo=False, update_pose=False):
    joints = [kinematics.set_leg_position(i + 1, position)
              for i, position in enumerate(new_pose)]
    joints = list(itertools.chain.from_iterable(joints))  # 18 radians
    joints_data = [[j, r] for j, r in zip(list(range(1, 19)), joints)]
    if not pseudo:
        self.joints_state = self.joint_control.set_multi_joints(
            duration, joints_data, self.joints_state)
    if update_pose:
        self.pose = tuple(map(tuple, new_pose))
```

**In prose.** For each of six feet, `kinematics.set_leg_position` (inside `kinematics.so`) returns three joint radians. Eighteen numbers. `JointControl` maps kinematic joint id → physical bus id, radians → pulse, and publishes **one** `ServosPosition` with `duration = 1.0`. The STM32 then takes a full second to arrive. The 20 ms loop does **not** cut the stand into 50 frames.

`reset_all_*_gen()` then empties walk and lean inboxes so a leftover gait cannot fight the stand.

`JointControl` conversion (kinematic id is not bus id):

```python
# ros2_ws/src/driver/kinematics/kinematics/x_joint_control.py:67-109
# for each (joint_id, radians):
#   servo_id = SERVOS[joint_id]['id']   # physical bus
#   pos = ...radians to 0–1000...
set_servo_position(self.joints_pub, duration, servos_data)
```

```python
# ros2_ws/src/driver/servo_controller/servo_controller/bus_servo_control.py:8-19
msg.duration = float(duration)
msg.position_unit = "pulse"
pub.publish(msg)   # topic servo_controller
```

### 6.2 Jobs B, C, D during halt and during the dance

After job A, the stand has cleared the generators. Jobs B and C see nothing. Job D checks `if pose is not self.pose` — identity of the Python object. After job A, `self.pose` already *is* that pose, so job D does not publish again.

Then:

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:266-268
if (no generator in flight):
    self.stopped.set()    # lets stop_running() return
time.sleep(0.02)
```

**What the 20 ms tick is doing during the dance.** Waking, seeing an empty inbox, sleeping. It does **not** publish servos. That is the property that lets the pianist own `/servo_controller`.

If someone queued a walk or a lean in the middle of the dance, job C or B would start publishing again and the dance would fight. Halt exists to prevent that.

---

## 7. Body lean (the other “pose transformation”)

The dance does not call this. It is here because the Python name “pose transformation” is this generator, not the dance.

**Who commands it.** Anyone who publishes `kinematics_msgs/TransformEuler` on `/controller/pose_transform_euler`. The receptionist stacks those deltas, clamps them, and asks the worker to start a generator:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:101-126
self.step_controller.transform_pose_euler(
    translation, "xyz", rotation, msg.duration, degrees=False)
```

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:398-414
generator = PoseTransformer(PoseTransformerParams(
    translation=translate, rotation=r, duration=duration))
generator.send(None)                    # prime the generator
self.new_pose_transformer = generator   # worker will drive it
```

**What the generator does.** Split `duration` into 20 ms frames. Each frame: add a fraction of the translation and of the xyz Euler angles, keep the feet on the floor, compute new foot xyz.

```python
# ros2_ws/src/driver/controller/controller/pose_transformer.py:23-71
div_num = max(math.ceil(params.duration / 0.02), 1)
# each tick of the worker:
cur_pose = kinematics_calculate.transform_euler(...)
yield cur_pose, out_tranform, False     # not the last frame
# last frame:
yield final_pose, final_transform, True
```

**How that meets the 20 ms loop.** Job B does one `send((pose, transform))` per tick. Job D sees a new pose object and calls `set_pose_base(..., duration=0.02)`. A 0.4 s lean is about twenty IK messages, each “get here in 20 ms.” Opposite of the stand: the stand is **one** 1.0 s message; a lean is **many** 0.02 s messages.

`/controller/set_pose_euler` is the absolute version (`absolutely=True`): the numbers are a target body pose, not a delta. The generator subtracts the current stored transform first.

---

## 8. Cueing the dance file

After Halt (and after Speak, which we skip), the director calls `start_dance()`:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:591-618
void start_dance() {
  interfaces::msg::RunActionSet msg;
  msg.action_path = action_name_;   // "master_dance" from YAML
  msg.interrupt = true;
  run_actionset_pub_->publish(msg);
  // music starts here; out of scope
  phase_ = Phase::Dance;
  saw_action_running_ = false;
  action_complete_ = false;
}
```

**Who commanded the dance.** The director. **What the postcard contains.** A string `action_path` and a bool `interrupt`. Type: `ros2_ws/src/interfaces/msg/RunActionSet.msg`. The string is a **basename**: `master_dance` means the file `/home/ubuntu/software/actionset_editor/ActionGroups/master_dance.d6a`. No folder, no `.d6a` suffix.

**Who receives it.** The receptionist, subscription `~/run_actionset` which ROS expands to `/controller/run_actionset` because the node is named `controller` and the `~` means “private.”

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:51-52, 83
self.status_pub = self.create_publisher(Bool, 'action_complete', 1)
self.agc = ActionGroupController(
    self.create_publisher(ServosPosition, 'servo_controller', 1),
    '/home/ubuntu/software/actionset_editor/ActionGroups',
    self.status_pub)
self.create_subscription(RunActionSet, '~/run_actionset',
                         self.run_actionset_callback, 1)
```

**Two ROS names that look similar and are not.**

| Name in code | Expands to | Who writes it |
|---|---|---|
| `'~/run_actionset'` (has `~`) | `/controller/run_actionset` | director → receptionist |
| `'action_complete'` (no `~`) | `/action_complete` | pianist → director (and the receptionist’s own flag) |

`/controller/action_complete` is a different mailbox. Nobody writes it. That is why the director subscribes to `/action_complete`.

The receptionist’s callback:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:217-243
def run_actionset_callback(self, msg: RunActionSet):
    file_path = msg.action_path
    if file_path == 'stop':
        self.agc.stop_action_group()
        time.sleep(0.5)
        self.agc.run_action('init_pose')
    elif file_path in ('dance_1', 'dance_2', 'dance_3'):
        # forwarded to another node, not this dance
        ...
    else:
        if self.action_complete == True:
            self.agc.start_action_thread(file_path)
```

**What happens to `master_dance`.** It is not `stop` and not `dance_1/2/3`, so the `else` runs. If no group is already playing (`action_complete` is true), the pianist starts a background thread. `msg.interrupt` is currently ignored on this path. To abort, the director later sends `action_path = "stop"`.

```python
# ros2_ws/src/driver/servo_controller/servo_controller/action_group_controller.py:24-32
def start_action_thread(self, actNum, lock_servos=''):
    if self.running_action:
        return
    threading.Thread(target=self.run_action, args=(actNum, lock_servos),
                     daemon=True).start()
```

---

## 9. How the pianist applies the action set

This thread is **not** the 20 ms worker. It is a `while` loop over SQLite.

### 9.0 Why this can work without IK (the pianist does not invent poses)

IK posing and action-group posing live at **different heights**.

| | Named stand (`DEFAULT_POSE`) | Action group (`master_dance.d6a`) |
|---|---|---|
| What is stored | six foot **xyz in millimetres** | 24 servo **pulses 0–1000** |
| Who computes joint angles | `kinematics.set_leg_position` (IK) at play time | nobody at play time |
| What is published | pulses **computed now** from xyz | pulses **copied** from the file |

A pulse is the servo firmware’s coordinate: “horn position 500,” not “foot at x=164 mm.” The STM32 already knows how to go to a pulse. IK is only needed when the *programmer* thinks in feet. The pianist thinks in motors.

Those motor numbers got into the file **before** ROS ever plays it. Typical recording, Qt editor `software/actionset_editor/main.py`:

1. **Manual** — torque off, you bend a leg by hand (`angularReadback` path first unloads servos, lines 405–407).
2. **Read angle** — the editor asks each servo `getServoPulse(i)` and appends a table row (lines 412–418). That is a photograph of the 24 motors, not a geometric pose.
3. Or you drag a slider: `pulse_valuechange` sends that pulse live (`setServoPulse`, lines 332–335) and **Add action** stores the slider values.
4. **Save** writes the table as SQLite: `Time, Servo1 … Servo24` (lines 685–721).

`master_dance.d6a` on disk right now is exactly that photograph, 25 times. First row: `Time=500`, servos 1–18 all pulse `500`, arm `500,720,130,150,500,500`. The pianist will send those integers again. It never asks “where is the foot.”

So the low-level look is the point: **playback is a tape recorder, not a geometer.** The person (or the slider) who recorded the row already solved “what the legs should look like,” by posing the real robot. IK would only be required if we wanted to *generate* those pulses from millimetres at run time. We do not.

The cost of skipping IK: the file has no idea whether three feet are on the floor. If a recorded row lifts too many legs, she falls. That is why halt to `DEFAULT_POSE` happens first, and why the first/last rows of a dance should be a stand.

File on disk: `/home/ubuntu/software/actionset_editor/ActionGroups/master_dance.d6a`  
Table `ActionGroup`: `Index`, `Time` (milliseconds), `Servo1` … `Servo24` (pulse 0–1000).  
At the time of writing: 25 rows, 14300 ms total.

```
for each row in the file:
    publish 24 pulses, duration = Time/1000 seconds
    sleep that same Time/1000
    (STM32 is sliding the servos during the sleep)
when the cursor runs off the end:
    publish /action_complete = true
```

The code:

```python
# ros2_ws/src/driver/servo_controller/servo_controller/action_group_controller.py:37-88
def run_action(self, actNum, lock_servos=''):
    actNum = os.path.join(self.action_path, actNum + ".d6a")
    self.running_action = True
    # /action_complete = false   →  "the roll has started"
    cu.execute("select * from ActionGroup")
    while True:
        act = cu.fetchone()
        if self.stop_running:      # set if someone sent action_path "stop"
            break
        if act is not None:
            msg.position_unit = 'pulse'
            msg.duration = float(act[1]) / 1000.0
            # Servo1..Servo24 → bus ids 1..24
            self.servo_controller_pub.publish(msg)
            time.sleep(float(act[1]) / 1000.0)
        else:
            # /action_complete = true  →  "the roll has finished"
            break
```

**What one row means.** `Time` is how long the STM32 should take to **arrive** at that keyframe from the previous pulses. It is not a pause after the pose. The Python `sleep` exists so the next row is not sent before that arrival time. There is no extra 20 ms subdivision unless the row’s `Time` is 20.

**What `/action_complete` false then true means.** `false` = this thread is running. `true` = the cursor fell off the end (or we are about to exit). The director insists on seeing false **before** it treats true as “this dance ended,” so a leftover true from the previous file cannot stop the wait on the first 50 ms tick:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:497-507, 621-622
void on_action_complete(...) {
  action_complete_ = msg->data;
  if (!msg->data) saw_action_running_ = true;
}
// done when (we saw running AND now complete) OR 16.5 s elapsed
```

**Abort.** If someone says stop during Dance, the director publishes `action_path = "stop"`. The receptionist stops the cursor and plays `init_pose.d6a` (a one-row stand of **pulses**, still not IK).

---

## 10. Last millimetres: from `/servo_controller` to the motors

Both the IK stand and every dance row become the same ROS type on the same topic.

```
worker or pianist
        v
/servo_controller     servo_controller_msgs/ServosPosition
        v
/controller_manager   event-driven (a message arrived — not a 20 ms servo clock)
        v
/ros_robot_controller/bus_servo/set_position
        v
USB packet to STM32   duration converted to integer milliseconds
        v
servos slide
```

**`controller_manager` is not ticking poses.** Its servo callback runs when a message appears:

```python
# ros2_ws/src/driver/servo_controller/servo_controller/controller_manager.py:51, 71-80
self.create_subscription(ServosPosition, 'servo_controller',
                         self.servo_controller_callback, 1)
def servo_controller_callback(self, msg):
    if msg.position_unit == 'pulse':
        self.servo_manager.set_position(msg.duration, data.position)
```

The 20 ms loop in **this** file only republishes `/joint_states` for TF (`controller_manager.py:100-118`). That is RViz, not the body.

Duration is clamped. This is the hardware 20 ms floor:

```python
# ros2_ws/src/driver/servo_controller/servo_controller/servo_controller.py:82-83
duration = 0.02 if duration < 0.02 else 30 if duration > 30 else duration
```

The serial node copies id and pulse into a list and calls the SDK:

```python
# ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py:119-124
self.board.bus_servo_set_position(msg.duration, data)
```

```python
# ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py:418-424
duration = int(duration * 1000)   # seconds → ms
# packet: command 0x01, duration lo/hi, count, then (id, pulse) pairs
self.buf_write(...)
```

**What the STM32 does with that.** Slide each listed servo from its current pulse to the new pulse in `duration` ms. If a new packet arrives early, the slide restarts from here. That is the fight if the 20 ms worker and the pianist both publish.

---

## 11. Two numberings

IK uses kinematic joint ids 1–18 (`SERVOS` in `ros2_ws/src/driver/kinematics/kinematics/config.py`). Action groups use **physical bus ids**. `JointControl` translates. The pianist does not.

| Leg | IK `leg_id` in `DEFAULT_POSE` | Bus ids coxa, femur, tibia |
|---|---|---|
| LF left front | 1 | 5, 3, **1** |
| LM left middle | 2 | 11, 9, 7 |
| LR left rear | 3 | 17, 15, 13 |
| RR right rear | 4 | 18, 16, 14 |
| RM right middle | 5 | 12, 10, 8 |
| RF right front | 6 | 6, 4, 2 |
| Arm | — | 19–24 |

`.d6a` column `Servo1` is bus 1 = LF **tibia**. `set_pose_base` joint 1 is LF **coxa** in kinematic space, which maps to bus 5. Mixing those is how a dance becomes a collapse.

---

## 12. The same story, in time

| When | Who | What, in the language of this page |
|---|---|---|
| 0 | director, first Halt tick | Commands halt. Sends Twist 0, Traveling gait 0, Traveling gait -2. |
| 0 | receptionist | Twist 0 would start a speed-0 walk if it were alone. gait 0 waits until the worker is idle. gait -2 writes `DEFAULT_POSE` into the worker’s inbox. |
| next 20 ms tick | worker, job A | IK six feet, one servo message, duration 1.0 s. Clears walk/lean. |
| 0 … 1.0 s | STM32 | Slides to the stand. |
| 0.4 s | director | Halt wait is over. Speak (skipped). Then `start_dance()`. |
| that moment | director → receptionist | Postcard: play `master_dance`. |
| pianist t = 0 | pianist | `/action_complete = false`. First row published. |
| each row | pianist + STM32 | Pulses + duration; Python sleeps; board slides. Worker is idle. |
| 14.3 s | pianist | `/action_complete = true`. |
| next 50 ms tick | director | Saw false then true. Back to Idle. |

If `/action_complete` never goes true, the director gives up at 16.5 s (`dance_timeout_s`). That timeout is a backstop, not the length of the dance.

---

## 13. Is IK really computed every 20 ms? (no micrometres, not a hard job)

Short answer: **not for the dance, not for the halt stand, and never in micrometres.** Inverse kinematics of one 3-joint insect leg is a handful of trig functions. Fifty times a second on a Jetson is idle time.

**What “IK” means here.** Given one foot’s xyz in **millimetres**, compute three joint **radians**. That is `kinematics.set_leg_position` inside `kinematics.so` (compiled). Six legs = six independent 3-DOF solves. There is no iterative optimiser, no whole-body Jacobian, no micrometre mesh.

**When it actually runs**

| What she is doing | IK every 20 ms? | What really happens |
|---|---|---|
| Dance (`master_dance.d6a`) | **No. Zero times.** | Pianist copies stored pulses. Worker is idle. |
| Halt stand (`DEFAULT_POSE`) | **Once**, not every tick | One `set_pose_base(..., duration=1.0)`. STM32 slides for a second. |
| Body lean (`PoseTransformer`) | Yes, for the duration of the lean | Each tick: new foot xyz, then 6× IK, publish `duration=0.02`. A 0.4 s lean ≈ 20 solves, then it stops. |
| Walk (gait) | Yes, while walking | Foot xyz for a whole step is **precomputed** (`move.py` `set_step_mode` in a batch). Each 20 ms tick only **indexes** that table, then still calls `set_pose_base` so xyz → radians. When walking stops, IK stops. |

Halt, in code: job A calls `set_pose_base` **once** (`step_controller.py:138-150`), then later ticks see an empty inbox.

Walk, in code: heavy gait geometry once per step —

```python
# ros2_ws/src/driver/controller/controller/move.py:114-133
for i in range(6):
    ps = kinematics.set_step_mode(...)   # a whole part of the step, many frames
    poses.append(ps)
# then every 20 ms:
out_pose = poses[part_index][sub_index]  # table lookup, not a new plan
```

— then `set_pose_base` still does 6× `set_leg_position` so the servos get radians (`step_controller.py:341`). That second step is cheap.

**Not micrometres.** `DEFAULT_POSE` is millimetres (LF foot about `(+164, +141, −70)`). A typical stride is tens of millimetres over about 1 s. At 50 Hz that is **about a millimetre per tick**, not a micrometre. One servo pulse is 0.24° (1000 pulses over 240°). At a ~100 mm shin that is roughly **0.4 mm at the foot** — coarser than a millimetre. The stack cannot even *command* micrometres.

**Why 20 ms is an easy job.** Closed-form 3-DOF IK is microseconds of CPU. Industrial arms run IK at kilohertz. 50 Hz is a leisurely rate chosen so the STM32 has 20 ms to slide the servos, not because the Jetson is busy. The expensive thing on this robot is never the math; it is fighting two publishers on `/servo_controller`.

---

## 14. Every “20 ms” you will meet

They are different clocks. Posing uses some of them and not others.

| 20 ms | File:line | Whose clock | Does it move the body? |
|---|---|---|---|
| `time.sleep(0.02)` | `step_controller.py:268` | worker loop | Only if job A/B/C/D published this tick |
| `duration=0.02` | job D in the same loop | one IK **frame** (leans and gaits) | Yes, one 20 ms slide |
| `duration/0.02` | `pose_transformer.py:23` | how many lean frames | Indirectly |
| clamp `duration >= 0.02` | `servo_controller.py:83` | hardware minimum | Yes: shorter requests become 20 ms |
| `time.sleep(0.02)` | `controller_manager.py:118` | `/joint_states` for RViz | No |
| `.d6a` `Time` minimum 20 | Qt editor | shortest legal keyframe | Only if the file uses 20 |

The director’s clock is **50 ms**, not 20. The pianist’s clock is **each row’s `Time`**.

---

## 15. If you remember only this

The director never moves a servo. It sends postcards to the receptionist.

**Halt** is a phase, not a message. Its content is three postcards: a zero walking speed, “stop stepping” (`gait = 0`), and “named stand” (`gait = -2` → `DEFAULT_POSE`). The receptionist writes the stand into the worker. The worker, on one 20 ms tick, IK’s the six feet and asks the STM32 for a **one-second** slide.

**The dance** is a different postcard: “play the file named `master_dance`.” The receptionist starts the pianist. The pianist is a file cursor. Each row is raw pulses and a duration. The 20 ms worker is idle on purpose so it does not fight.

**Pose transformation** in the Python source is a third thing: feet planted, body leaned a little every 20 ms. This dance never queues it.

From `/servo_controller` downward there is only one pipe. Whoever publishes there owns the legs until the next message.
