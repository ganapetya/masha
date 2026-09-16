# How posing works on Masha

This is a reading note for **this robot**. After you read it you should be able to follow one stand and one dance from the first ROS message to the last servo pulse, and know which clock is ticking.

Voice is out of scope. Treat `masha_interaction_node` as a **stage manager that already decided to pose**. How it heard the sentence is in `steps/MASTER_DIALOG_DANCE.md`. How a `.d6a` is recorded is in `steps/DANCE_ACTION_SETS.md`. How walking works is in `steps/MASHA_GAITS.md`.

Copies: `~/steps/DANCE_ACTION_SETS_DETAILS.md`  
Worked example: `master_dance` as cued by `ros2_ws/src/proud_up/src/masha_interaction_node.cpp`.

---

## 1. Three motion machines (posing is two of them)

Masha does not have one “pose API”. Three machines share the same last millimetres of cable to the STM32. They must not run at the same time.

| Machine | What a pose is | Who computes it | Typical trigger |
|---|---|---|---|
| **Built-in / IK pose** | Six foot-tip xyz in millimetres. Inverse kinematics → joint radians → pulses. | `StepController` 20 ms loop | `/controller/traveling` gait `-2` (`DEFAULT_POSE`), or `~/set_pose_1` |
| **Body transform** | Same six feet, still planted. The body is translated and rotated over them, one 20 ms frame at a time. | `PoseTransformer` generator inside that same loop | `/controller/pose_transform_euler`, `/controller/set_pose_euler` |
| **Action group** | Open-loop **pulses** from a `.d6a` SQLite file. No IK, no support polygon. | `ActionGroupController` background thread | `/controller/run_actionset` |

The dance we built uses **machine 1 then machine 3**:

1. Halt the gait and blend to `DEFAULT_POSE` (IK, 20 ms loop).
2. Play `master_dance.d6a` (pulses, one keyframe after another).

It does **not** use `PoseTransformer`. That generator is the other posing path (body lean). It is documented in §7 so the name “pose transformation” in the Python files is not a mystery.

---

## 2. The picture

Two ROS processes, four clocks. Time flows down.

```
masha_interaction_node          50 ms wall timer  (tick)
        │
        │  Halt: Twist 0, Traveling gait=0, Traveling gait=-2
        │  Dance: RunActionSet { action_path: master_dance }
        ▼
process move_controller
  node /controller              MoveController          public topics
  node /step_controller         StepController          20 ms IK loop
        │
        ├─ IK pose / body transform
        │     set_pose_base → JointControl.set_multi_joints
        │           duration = 1.0 s  (stand)  or  0.02 s  (each IK frame)
        │
        └─ action group (separate thread, not the 20 ms loop)
              ActionGroupController.run_action
                    one ServosPosition per .d6a row
                    duration = row.Time / 1000
                    then sleep that same duration
        ▼
topic  /servo_controller        servo_controller_msgs/ServosPosition
        ▼
node /controller_manager        event-driven callback (not a 20 ms servo tick)
        duration clamped to [0.02, 30] s
        ▼
topic  /ros_robot_controller/bus_servo/set_position
        ▼
STM32 on /dev/ttyACM0           interpolates current pulse → target pulse
                                over `duration` milliseconds
        ▼
bus servos 1–24
```

`controller_manager` also publishes `/joint_states` every 20 ms. That is TF for RViz. It is **not** the pose clock.

The STM32 interpolator is the only place a “20 ms move” is a hardware rule: durations below 0.02 s are raised to 0.02 s.

---

## 3. What inits the process (no voice)

The interaction node is launched by `ros2_ws/src/proud_up/launch/masha_interaction.launch.py`. YAML: `ros2_ws/src/proud_up/config/masha_interaction.yaml`.

On construction it creates the three publishers the body will listen to, and a **50 ms** timer that drives the phase machine:

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

Defaults (overridden by the YAML):

| Parameter | Default | Role for posing |
|---|---|---|
| `traveling_topic` | `/controller/traveling` | halt / stand |
| `cmd_vel_topic` | `/controller/cmd_vel` | kill a leftover walk generator |
| `run_actionset_topic` | `/controller/run_actionset` | start `master_dance` |
| `action_complete_topic` | `/action_complete` | false = roll running, true = roll finished |
| `action_name` | `master_dance` | basename of the `.d6a` |
| `halt_wait_s` | `0.4` | wait after stand is *sent*, not after it *arrives* |
| `dance_timeout_s` | `16.5` | backstop if `/action_complete` never goes true |

The 50 ms timer is the stage manager. It does not move servos. It publishes postcards and watches flags.

Phases that matter for posing (Speak is only a gate; we do not describe audio):

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:352-356
enum class Phase {
  Idle,
  Halt,
  Speak,
  Dance,
};
```

Once `phase_` is `Halt`, every 50 ms `tick()` copies flags under the mutex, drops the lock, then calls `tick_halt` / `tick_speak` / `tick_dance`:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:509-540
void tick() {
  // Copy the note, drop the key, then act.
  // ...
  switch (phase) {
    case Phase::Idle:  break;
    case Phase::Halt:  tick_halt(halt_sent, abort); break;
    case Phase::Speak: tick_speak(abort); break;
    case Phase::Dance: tick_dance(dance_sent, saw_running, action_complete, abort); break;
  }
}
```

Posing starts on the first `Halt` tick that has not yet sent the stand.

---

## 4. Halt: standing still is not walking at speed zero

First Halt tick calls `halt_legs()` once, then waits `halt_wait_s` (0.4 s):

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:543-553
void tick_halt(bool halt_sent, bool abort) {
  if (abort) { enter_idle("abort before speak"); return; }
  if (!halt_sent) {
    halt_legs();
    std::lock_guard<std::mutex> lock(mutex_);
    halt_sent_ = true;
    phase_started_ = now();
    return;
  }
  if (elapsed_s() < halt_wait_s_) { return; }
  // ... then Speak, then start_dance()
}
```

Three postcards, this order:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:660-686
void halt_legs() {
  geometry_msgs::msg::Twist zero;
  cmd_vel_pub_->publish(zero);          // 1. leftover walk must not keep a generator

  kinematics_msgs::msg::Traveling stop;
  stop.gait = 0;
  stop.time = 1.0f;
  stop.interrupt = true;
  traveling_pub_->publish(stop);        // 2. stop the stepping loop

  kinematics_msgs::msg::Traveling stand;
  stand.gait = -2;
  stand.time = 1.0f;
  stand.interrupt = true;
  traveling_pub_->publish(stand);       // 3. blend to DEFAULT_POSE
}
```

Why three:

- A **zero Twist** on `/controller/cmd_vel` still starts `CmdVelGenerator` at speed 0. She would march in place. The Twist is sent only to replace a leftover non-zero walk; the stand that follows is what actually stops her.
- `gait == 0` waits until the 20 ms loop is idle, then holds the current six-foot pose for `msg.time` (1.0 s) and drops every generator.
- `gait == -2` queues `DEFAULT_POSE`. It is last so the stand wins if both Traveling messages are processed in one burst.

`MoveController` is the mailbox:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:155-189
def set_traveling_callback(self, msg: Traveling):
    if msg.gait > 0:
        self.step_controller.set_step_mode(...)   # walk — not this page
    else:
        if msg.gait == 0:
            self.step_controller.stop_running(
                timeout=None,
                callback=lambda: self.step_controller.set_pose(None, None, msg.time)
            )
        if msg.gait == -1:
            self.step_controller.set_build_in_pose('SLAM_POSE', msg.time)
        elif msg.gait == -2:
            self.step_controller.set_build_in_pose('DEFAULT_POSE', msg.time)
            self.current_trans = [0.0, 0.0, 0.0]
            self.current_rot = [0.0, 0.0, 0.0]
```

`stop_running(timeout=None)` blocks that callback until the 20 ms loop has no generator left (`stopped` Event). Then the lambda queues “hold current pose for 1.0 s”. The later gait `-2` callback overwrites that slot with `DEFAULT_POSE` — the stand is what the loop will actually apply.

`set_build_in_pose` only **queues**. It does not talk to servos on the ROS callback thread:

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:366-377
def set_build_in_pose(self, pose_name, duration, interrupt=True):
    with self.lock:
        pose = getattr(build_in_pose, pose_name)
        transform = getattr(build_in_pose, pose_name + '_TRANSFORM')
        self.new_pose_setter = (pose, transform, duration)
```

`DEFAULT_POSE` is six foot tips, millimetres, body frame +X head, +Y left, +Z up, z negative (feet below the body):

```python
# ros2_ws/src/driver/controller/controller/build_in_pose.py:15-20 and 62
DEFAULT_POSE = ((INITIAL_X + X1, INITIAL_Y + Y1 - 20, -INITIAL_HEIGHT),  # LF ≈ (+164, +141, −70)
                (0.0, INITIAL_Y + Y2, -INITIAL_HEIGHT),                  # LM ≈ (0, +184, −70)
                (-INITIAL_X - X1, INITIAL_Y + Y1 - 20, -INITIAL_HEIGHT), # LR
                (-INITIAL_X - X1, -INITIAL_Y - Y1 + 20, -INITIAL_HEIGHT),# RR
                (0.0, -INITIAL_Y - Y2, -INITIAL_HEIGHT),                 # RM
                (INITIAL_X + X1, -INITIAL_Y - Y1 + 20, -INITIAL_HEIGHT)) # RF
DEFAULT_POSE_TRANSFORM = (0, 0, 130), (0, 0, 0)
```

Leg order in that tuple is IK `leg_id` 1…6 = LF, LM, LR, RR, RM, RF. Physical bus IDs are a different numbering (see §10).

---

## 5. The 20 ms tick (IK posing)

Boot starts one process that is **two ROS nodes**:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:276-282
def main(args=None):
    node = MoveController('controller')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.step_controller)
    executor.spin()
```

`StepController.__init__` starts a **daemon thread** whose only job is this loop. ROS callbacks never run it.

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:85-87
self.loop_thread = threading.Thread(target=self.loop, daemon=True)
self.loop_enable = True
self.loop_thread.start()
```

One pass of `loop()` is one tick. The sleep at the bottom is 20 ms, so the intended rate is 50 Hz. The thread is `renice -19` so it is less likely to slip.

Each tick does **four jobs, always in this order**. Pose-set wins: if a new built-in pose is queued, everything else is dropped.

### 5.1 Job A — apply a queued built-in pose (the stand)

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:138-150
if self.new_pose_setter is not None:
    self.cur_pose_setter = self.new_pose_setter
    self.new_pose_setter = None
if self.cur_pose_setter is not None:
    pose, transform, duration = self.cur_pose_setter
    if pose is None or transform is None:
        self.set_pose_base(self.pose, duration, update_pose=True)   # hold current
    else:
        self.set_pose_base(pose, duration, update_pose=True)       # DEFAULT_POSE, duration=1.0
        self.transform = transform
    self.reset_all_cur_gen()
    self.reset_all_new_gen()
```

For gait `-2` this is one call: IK all six legs, convert 18 joints to pulses, publish **one** `ServosPosition` with `duration=1.0`, remember the new `self.pose`, then clear gait and transform generators so a leftover walk cannot fight the stand.

`set_pose_base` is the only IK-to-servo function the loop uses:

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:331-350
def set_pose_base(self, new_pose, duration, pseudo=False, update_pose=False):
    joints = [kinematics.set_leg_position(i + 1, position) for i, position in enumerate(new_pose)]
    joints = list(itertools.chain.from_iterable(joints))
    joints_data = [[j, r] for j, r in zip(list(range(1, 19)), joints)]
    if not pseudo:
        self.joints_state = self.joint_control.set_multi_joints(duration, joints_data, self.joints_state)
    if update_pose:
        self.pose = tuple(map(tuple, new_pose))
```

`kinematics.set_leg_position` (inside `kinematics.so`) returns three joint **radians** per leg. Kinematic joint ids 1…18 are not bus ids. `JointControl` maps them:

```python
# ros2_ws/src/driver/kinematics/kinematics/x_joint_control.py:67-109
def set_multi_joints(self, duration, data, joints_state=None):
    servos_data = []
    for joint_id, radians in data:
        servo = SERVOS[joint_id]
        servo_id = servo['id']          # physical bus id
        # radians → pulse 0–1000 using center, ticks, direction, offset
        pos = int((real_radians - (-max_radians / 2)) / max_radians * ticks + min_ticks)
        servos_data.append([servo_id, pos])
    set_servo_position(self.joints_pub, duration, servos_data)
```

`set_servo_position` publishes on topic `servo_controller` with `position_unit = "pulse"`:

```python
# ros2_ws/src/driver/servo_controller/servo_controller/bus_servo_control.py:8-19
def set_servo_position(pub, duration, positions):
    msg = ServosPosition()
    msg.duration = float(duration)
    # ... id, position for each servo ...
    msg.position_unit = "pulse"
    pub.publish(msg)
```

The STM32 then blends from wherever the servos are now to those pulses in **one second**. The 20 ms loop does **not** slice `DEFAULT_POSE` into 50 frames. One message, hardware interpolation.

`halt_wait_s` is 0.4 s and the stand duration is 1.0 s. Speak (and then the dance) can start while the stand is still travelling. That is why the first row of `master_dance.d6a` should itself be a stand, not a leap.

### 5.2 Job B — body transform (PoseTransformer)

Only if `new_pose_transformer` was queued. The dance does not queue one. See §7.

### 5.3 Job C — gait generator

Only if a walk is running. Halt just cleared it. During the dance this slot stays `None`.

### 5.4 Job D — apply a transformed or gait pose, then sleep

If Job B produced a new 6-foot pose object (`pose is not self.pose`), send it with `duration=0.02` (one frame). If a gait pose exists, send that instead (also 0.02 s, or 0.05 s on a large cmd_vel splice). Then:

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:266-268
if self.cur_moving_generator is None and self.cur_pose_setter is None \
        and self.cur_pose_transformer is None and self.cur_actionset_runner is None:
    self.stopped.set()
time.sleep(0.02)
```

When idle (after the stand has been applied, before/during the dance):

- no pose setter, no transformer, no gait
- `pose is self.pose` → skip Job D’s publish
- `stopped` is set, so `stop_running` can return
- the thread still wakes every 20 ms and does nothing to the servos

That is the important property: **the 20 ms loop does not fight the action group**, as long as nobody queues a new pose or gait while the `.d6a` plays.

---

## 6. Cueing the action set

After Halt (and after Speak, which we skip), `start_dance()` publishes one postcard. It is a topic, not a service.

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:591-618
void start_dance() {
  interfaces::msg::RunActionSet msg;
  msg.action_path = action_name_;   // "master_dance"
  msg.interrupt = true;
  run_actionset_pub_->publish(msg);

  // music starts here; out of scope

  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::Dance;
  dance_sent_ = true;
  saw_action_running_ = false;
  action_complete_ = false;
  phase_started_ = now();
}
```

Message type:

```
# ros2_ws/src/interfaces/msg/RunActionSet.msg
string action_path
bool interrupt
```

`action_path` is the **basename** of a file under `/home/ubuntu/software/actionset_editor/ActionGroups/`. No folder, no `.d6a`. Reserved names that are not this dance: `stop`, `dance_1`, `dance_2`, `dance_3`.

ROS 2 name expansion, node name `controller`, namespace `/`:

```
'~/run_actionset'   →  /controller/run_actionset     (tilde = private)
'action_complete'   →  /action_complete              (no tilde = namespace only)
```

That is why the C++ node publishes `/controller/run_actionset` and subscribes `/action_complete`. `/controller/action_complete` is empty.

`MoveController` already owns the pianist:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:51-52, 83
self.status_pub = self.create_publisher(Bool, 'action_complete', 1)
self.agc = ActionGroupController(
    self.create_publisher(ServosPosition, 'servo_controller', 1),
    '/home/ubuntu/software/actionset_editor/ActionGroups',
    self.status_pub)
self.create_subscription(RunActionSet, '~/run_actionset', self.run_actionset_callback, 1)
```

The callback:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:217-243
def run_actionset_callback(self, msg: RunActionSet):
    file_path = msg.action_path
    if file_path == 'stop':
        self.agc.stop_action_group()
        self.perfrom_actions_pub.publish(msg)
        time.sleep(0.5)
        self.agc.run_action('init_pose')
    elif file_path in ('dance_1', 'dance_2', 'dance_3'):
        if self.action_complete == True:
            self.perfrom_actions_pub.publish(msg)
    else:
        if self.action_complete == True:
            self.agc.start_action_thread(file_path)
```

`master_dance` takes the `else` branch. A second group will not start until `/action_complete` is true (`self.action_complete`). `interrupt` on the message is currently ignored here; `stop` is the abort path.

`start_action_thread` refuses to start if a group is already running, then spawns a daemon thread:

```python
# ros2_ws/src/driver/servo_controller/servo_controller/action_group_controller.py:24-32
def start_action_thread(self, actNum, lock_servos=''):
    if self.running_action:
        return
    action_thread = threading.Thread(target=self.run_action, args=(actNum, lock_servos))
    action_thread.daemon = True
    action_thread.start()
```

---

## 7. How the action set is applied (the piano roll)

This thread is **not** the 20 ms loop. It is a cursor over SQLite.

File: `/home/ubuntu/software/actionset_editor/ActionGroups/master_dance.d6a`  
Table `ActionGroup`: `Index`, `Time` (ms), `Servo1` … `Servo24` (pulse 0–1000).  
On disk at the time of writing: **25 rows, 14300 ms**.

```python
# ros2_ws/src/driver/servo_controller/servo_controller/action_group_controller.py:37-88
def run_action(self, actNum, lock_servos=''):
    actNum = os.path.join(self.action_path, actNum + ".d6a")
    # ...
    self.running_action = True
    if self.status_pub is not None:
        msg = Bool(); msg.data = False
        self.status_pub.publish(msg)          # /action_complete = false  (roll started)

    ag = sql.connect(actNum)
    cu = ag.cursor()
    cu.execute("select * from ActionGroup")

    while True:
        act = cu.fetchone()
        if self.stop_running:                 # set by action_path "stop"
            break
        if act is not None:
            msg = ServosPosition()
            msg.position_unit = 'pulse'
            msg.duration = float(act[1]) / 1000.0   # Time column, seconds
            for i in range(0, len(act) - 2, 1):
                servo = ServoPosition()
                servo.id = i + 1                    # physical bus id 1..24
                servo.position = float(act[2 + i])
                data.append(servo)
            msg.position = data
            self.servo_controller_pub.publish(msg)
            time.sleep(float(act[1]) / 1000.0)      # wait for STM32 to finish this row
        else:
            if self.status_pub is not None:
                msg = Bool(); msg.data = True
                self.status_pub.publish(msg)        # /action_complete = true  (roll finished)
            break
```

What one row does:

1. Publish all 24 pulses with `duration = Time/1000`.
2. Sleep that same time. The sleep is how the Python cursor stays in step with the STM32 interpolator. It is not a second interpolation.
3. Next row.

There is no quintic, no IK, no 20 ms subdivision in this thread. A 700 ms row is one message and a 700 ms sleep. The board walks the pulses in between.

`Servo1` in the file is bus id 1 (LF tibia), not IK `leg_id` 1. Mixing those numberings is how a dance becomes a collapse. Mapping is in §10.

While this runs, the interaction node’s 50 ms `tick_dance` waits for **false then true** on `/action_complete`, so a leftover `true` from the previous group cannot end the dance on the first tick:

```cpp
// ros2_ws/src/proud_up/src/masha_interaction_node.cpp:497-507 and 621-622
void on_action_complete(...) {
  action_complete_ = msg->data;
  if (!msg->data) saw_action_running_ = true;   // must see "running" before "finished"
}
void tick_dance(...) {
  const bool done = (saw_running && action_complete) || elapsed_s() >= dance_timeout_s_;
```

Abort during Dance publishes `action_path = "stop"`, which stops the cursor and plays `init_pose.d6a` (a one-row stand, pulses, still not IK).

---

## 8. The other posing path: PoseTransformer (body lean)

This is what the Python files mean by “pose transformation”. The dance does not call it. Anything that publishes `TransformEuler` on `/controller/pose_transform_euler` does.

`MoveController` stacks relative deltas, clamps them, then queues a generator:

```python
# ros2_ws/src/driver/controller/controller/move_controller.py:101-126
def pose_transform_euler_callback(self, msg: TransformEuler):
    current_trans = [msg.translation.x, msg.translation.y, msg.translation.z]
    current_rot = [msg.rotation.x, msg.rotation.y, msg.rotation.z]
    # accumulate, clamp per axis, then:
    self.step_controller.transform_pose_euler(
        translation, "xyz", rotation, msg.duration, degrees=False)
```

```python
# ros2_ws/src/driver/controller/controller/step_controller.py:398-414
def transform_pose_euler(self, translate, axis, euler, duration, degrees=True):
    rotate = R.from_euler(axis, euler, degrees=degrees)
    r = rotate.as_euler('xyz', degrees=False)
    generator = PoseTransformer(PoseTransformerParams(
        translation=translate, rotation=r, duration=duration))
    generator.send(None)                       # prime
    self.new_pose_transformer = generator      # 20 ms loop will drive it
```

The generator splits `duration` into 20 ms frames:

```python
# ros2_ws/src/driver/controller/controller/pose_transformer.py:23-71
div_num = max(math.ceil(params.duration / 0.02), 1)
# ... convert absolute target to a delta if absolutely=True ...
org_pose, cur_transform = yield None
# each frame: lerp translation and xyz Euler, then
cur_pose = kinematics_calculate.transform_euler(
    org_pose, (nx, ny, nz), 'xyz', (nu, nv, nw), degrees=False)
yield cur_pose, out_tranform, False            # last_part=False
# last frame:
yield final_pose, final_transform, True        # last_part=True → loop drops the generator
```

`transform_euler` keeps the feet planted and moves the body. For each leg it calls `kinematics.transform_pose` in the `.so` with the same translation and quaternion:

```python
# ros2_ws/src/driver/kinematics/kinematics/kinematics_calculate.py:4-17
def transform_euler(pose, translate, axis, euler, degrees=True):
    quat = R.from_euler(axis, euler, degrees=degrees).as_quat()
    pose = tuple(kinematics.transform_pose(leg, pose[leg - 1], translate, quat)
                 for leg in range(1, 7))
    return pose
```

Back in the 20 ms loop, Job B does one `send((pose, transform))` per tick. Job D sees a new pose object and calls `set_pose_base(..., duration=0.02)`. So a 0.4 s lean is about 20 IK messages, each telling the STM32 “get here in 20 ms”. That is the opposite of the stand: the stand is **one** 1.0 s message; a body transform is **many** 0.02 s messages.

`absolutely=True` (`/controller/set_pose_euler`) means the numbers are a target pose of the body, not a delta. The generator subtracts the current `self.transform` first.

---

## 9. Last millimetres: controller_manager → STM32

Both IK poses and action-group rows arrive as `servo_controller_msgs/ServosPosition` on `/servo_controller`.

```python
# ros2_ws/src/driver/servo_controller/servo_controller/controller_manager.py:51, 71-80
self.create_subscription(ServosPosition, 'servo_controller', self.servo_controller_callback, 1)

def servo_controller_callback(self, msg):
    if msg.position_unit == 'pulse':
        for i in msg.position:
            if str(i.id) in positions:
                data.position.append(i)
        self.servo_manager.set_position(msg.duration, data.position)
```

This callback is **event-driven**. There is no 20 ms servo ticker here. The 20 ms thread in this file only republishes `/joint_states` for TF (`publish_joint_states`, line 100, `time.sleep(0.02)`).

Duration is clamped. That is the hardware 20 ms floor:

```python
# ros2_ws/src/driver/servo_controller/servo_controller/servo_controller.py:82-94
def set_position(self, duration, position):
    duration = 0.02 if duration < 0.02 else 30 if duration > 30 else duration
    msg = ServosPosition()          # ros_robot_controller_msgs this time
    msg.duration = float(duration)
    # clamp each pulse to 0–1000
    self.servo_position_pub.publish(msg)   # ros_robot_controller/bus_servo/set_position
```

The board node packs duration as integer milliseconds and writes the bus-servo packet:

```python
# ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py:119-124
def set_bus_servo_position(self, msg):
    data = []
    for i in msg.position:
        data.extend([[i.id, i.position]])
    if data:
        self.board.bus_servo_set_position(msg.duration, data)
```

```python
# ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py:418-424
def bus_servo_set_position(self, duration, positions):
    duration = int(duration * 1000)          # seconds → ms
    data = [0x01, duration & 0xFF, 0xFF & (duration >> 8), len(positions)]
    for i in positions:
        data.extend(struct.pack("<BH", i[0], i[1]))   # id, pulse
    self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
```

The STM32 (and the servo firmware) interpolate from the current pulse to the new pulse over that many milliseconds. Python does not. If you publish a new target before the previous duration has elapsed, the board starts a **new** interpolation from wherever the servo has reached. That is the fight you get if a gait generator is still ticking while a `.d6a` plays.

---

## 10. Two numberings (do not mix them)

IK pose uses kinematic joint ids 1–18 in `SERVOS` (`ros2_ws/src/driver/kinematics/kinematics/config.py`). Action groups use **physical bus ids**. `JointControl` translates; `ActionGroupController` does not.

| Leg | IK `leg_id` / `pose[]` | Kinematic joints (coxa, femur, tibia) | Bus ids (coxa, femur, tibia) |
|---|---|---|---|
| LF left front | 1 | 1, 2, 3 | **5, 3, 1** |
| LM left middle | 2 | 4, 5, 6 | **11, 9, 7** |
| LR left rear | 3 | 7, 8, 9 | **17, 15, 13** |
| RR right rear | 4 | 10, 11, 12 | **18, 16, 14** |
| RM right middle | 5 | 13, 14, 15 | **12, 10, 8** |
| RF right front | 6 | 16, 17, 18 | **6, 4, 2** |
| Arm + gripper | — | — | 19–24 |

`.d6a` column `Servo1` is bus 1 = LF tibia. `set_pose_base` joint 1 is LF coxa in kinematic space, which `SERVOS[1]['id']` maps to bus 5.

---

## 11. End-to-end timeline (`master_dance`)

Times are what the code asks for, not a stopwatch.

| t | Who | What |
|---|---|---|
| 0 | interaction `tick` 50 ms | `Halt`, first pass: `halt_legs()` |
| 0 | `/controller` | zero Twist → would start a speed-0 gait if it were alone |
| ~0 | `/controller` | `Traveling gait=0` → `stop_running` waits until 20 ms loop idle, queues hold-current |
| ~0 | `/controller` | `Traveling gait=-2` → `new_pose_setter = (DEFAULT_POSE, TRANSFORM, 1.0)` overwrites the hold |
| next 20 ms tick | `step_controller.loop` Job A | IK 6 legs, publish 18 pulses, **duration 1.0 s**, clear generators |
| 0 … 1.0 s | STM32 | blend to the stand |
| 0.4 s | interaction | `halt_wait_s` elapsed; Speak starts (audio skipped here) |
| Speak done | interaction | `start_dance()`: `RunActionSet master_dance` |
| same callback | `/controller` | `start_action_thread('master_dance')` |
| pianist t=0 | `ActionGroupController` | `/action_complete = false` |
| pianist each row | same thread | publish 24 pulses, `duration = Time/1000`, `sleep(Time/1000)` |
| 0 … 14.3 s | STM32 | interpolate each row; 20 ms loop is idle (no servo publishes) |
| 14.3 s | pianist | `/action_complete = true` |
| next 50 ms tick | interaction | saw false then true → Idle |

If `/action_complete` never goes true, `dance_timeout_s` (16.5 s) stops the wait anyway.

---

## 12. What “posing” means, in one page

When people say Masha poses, they can mean three different programs. The dance we built uses the first and the third.

**1. A named IK pose** (`DEFAULT_POSE`, `SLAM_POSE`, …)

- Trigger: `Traveling.gait == -2` or service `~/set_pose_1`.
- Clock: `StepController.loop`, 20 ms, but the stand itself is **one** `set_pose_base` with `duration=1.0`.
- Compute: six foot xyz → `kinematics.set_leg_position` → radians → `JointControl` pulses → `/servo_controller`.
- The 20 ms sleep is so the thread can pick up the *next* command. It does not chop the stand into frames.

**2. A body transform** (`PoseTransformer`)

- Trigger: `/controller/pose_transform_euler` or `/controller/set_pose_euler`.
- Clock: same 20 ms loop, **one IK frame per tick**, `duration=0.02` each.
- Compute: lerp translation + xyz Euler, `kinematics.transform_pose` per leg, feet stay down, body moves.
- This is the code’s phrase “pose transformation”. The dance node never queues it.

**3. An action group** (`master_dance.d6a`)

- Trigger: `/controller/run_actionset` with the basename.
- Clock: a **background thread** that sleeps each row’s `Time`. Not 20 ms unless a row is 20 ms.
- Compute: none. Pulses as recorded. STM32 interpolates over that row’s duration.
- Done signal: `/action_complete` false, then true.

The 20 ms numbers you will meet, and what they actually are:

| 20 ms | Where | What it is |
|---|---|---|
| `time.sleep(0.02)` | `step_controller.py:268` | IK / transform / gait loop period |
| `duration=0.02` | `set_pose_base` in Job D | one IK frame sent to the STM32 |
| `div_num = duration/0.02` | `pose_transformer.py:23` | how many body-lean frames |
| `duration = 0.02 if duration < 0.02` | `servo_controller.py:83` | hardware minimum move time |
| `time.sleep(0.02)` | `controller_manager.py:118` | `/joint_states` publish period (TF only) |
| `.d6a` `Time` minimum 20 | Qt editor | shortest legal keyframe |

For the dance: halt uses the IK-pose machine once (1.0 s hardware blend). The choreography is the action-group machine. The 20 ms loop keeps beating but, after the stand is applied, publishes nothing until someone queues a pose, a lean, or a gait again.

That is how posing works.
