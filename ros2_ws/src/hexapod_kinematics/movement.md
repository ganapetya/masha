# How a walk moves

Two clocks run at once. The hunter decides a body velocity every
50 ms. The leg thread plays one picture of the six feet every 20 ms.
The pictures are baked in advance, as a film of small frames. The
thread does not invent a new foot curve on each tick. It turns one
page.

This page is the order of those calls. Each actor in a diagram is
named with the file it lives in.

## The two clocks

| Clock | Period | Who starts it | What one tick does |
|---|---|---|---|
| Hunt | 50 ms | `control_timer_` in `proud_up/src/masha_hunter_node.cpp` | Copy sensors, call `tick()` in `proud_up/include/proud_up/hunter.hpp`, maybe publish one `Twist`. |
| Legs | 20 ms | `StepController.loop` in `driver/controller/controller/step_controller.py` | Take one already-baked frame and command the servos. Then `time.sleep(0.02)`. |
| Servo blend | 20 ms, or 50 ms on one splice frame | `JointControl.set_multi_joints` in `driver/kinematics/kinematics/x_joint_control.py` | How long the bus is told to spend reaching this pulse. The leg thread still wakes every 20 ms. |

The 50 ms of the hunt and the 50 ms of a servo blend are different
numbers that happen to match. The hunt clock publishes a velocity.
The blend clock is a duration written into one servo message.

A gait cycle is longer than either clock. The hunter's usual period
is 0.60 s. At 20 ms per frame that film has 30 pages
(`FollowGaitGenerator` in `move.py`: `period * 1000 / 20`).

## The cast

| Actor | File |
|---|---|
| `control_tick()` | `proud_up/src/masha_hunter_node.cpp` |
| `Hunter::tick()` | `proud_up/src/hunter.cpp` (declared in `proud_up/include/proud_up/hunter.hpp`) |
| `cmd_vel()` | `driver/controller/controller/step_controller.py` |
| `set_step_mode()` | same file |
| `loop()` | same file |
| `_pull_mini_step_input()` | same file |
| `_tick_legacy_gait()` | same file |
| `_tick_hunter_gait()` | same file |
| `_WalkState.after_accepted_gait_step()` | same file |
| `MovingGenerator` | `driver/controller/controller/move.py` |
| `CmdVelGenerator` | same file |
| `FollowGaitGenerator` | same file |
| `sample_follow_gait()` | same file |
| `Gait5Session` | `driver/controller/controller/leg_ik.py` |
| `approach_target()` | same file |
| `VendorLegIk` | same file |
| `HexapodLegIk` | same file |
| `kinematics.set_step_mode`, `cmd_vel_new_point`, `set_leg_position` | closed `kinematics.so`, called from `move.py` and `leg_ik.py` |
| `solve_pose()` | `hexapod_kinematics/src/leg_ik.cpp`, reached through `src/bindings.cpp` |
| `gate()` | `hexapod_kinematics/src/safety.cpp`, same binding |
| `JointControl.set_multi_joints` | `driver/kinematics/kinematics/x_joint_control.py` |

## 1. A 50 ms hunt tick only queues a film

`control_tick` does not choose foot positions. It publishes a
`Twist`. `cmd_vel()` builds a generator, primes it, and leaves it
in `new_moving_generator`. The feet stay where they are until the
20 ms loop reaches a step boundary and starts that film.

`SelectGait15` is a different message, sent once before the first
`Twist` of a bout. `set_step_mode(15)` stores `cmd_gait = 5`. Later
`Twist` messages then build `FollowGaitGenerator` instead of
`CmdVelGenerator`.

```mermaid
sequenceDiagram
    autonumber
    participant Hunt as control_tick() in masha_hunter_node.cpp
    participant Brain as Hunter::tick() in hunter.cpp
    participant Cmd as cmd_vel() in step_controller.py
    participant Film as FollowGaitGenerator in move.py
    participant Queue as new_moving_generator in step_controller.py

    Note over Hunt: Wall timer, every 50 ms
    Hunt->>Brain: latest pose and scan
    Brain-->>Hunt: LegCommand Twist, vx vy wz
    Hunt->>Cmd: publish /controller/cmd_vel
    Cmd->>Cmd: m/s to mm/s, gait = cmd_gait
    alt cmd_gait is 5, the hunter
        Cmd->>Film: construct FollowGaitGenerator
        Cmd->>Film: gen_advance_to_first_yield, run until the first yield
        Note over Film: Body is paused. No foot page exists yet.
        Cmd->>Queue: store this primed generator
    else any other cmd_gait
        Cmd->>Cmd: construct CmdVelGenerator in move.py
        Cmd->>Queue: store that primed generator
    end
    Note over Queue: This callback does not move a servo.
```

A joystick walk never enters that picture. `set_step_mode()` builds
the film directly. Gait 5 is still `FollowGaitGenerator`. Every
other gait is `MovingGenerator`, and that path is the one that
calls `kinematics.set_step_mode`.

```mermaid
sequenceDiagram
    autonumber
    participant Mode as set_step_mode() in step_controller.py
    participant Follow as FollowGaitGenerator in move.py
    participant Move as MovingGenerator in move.py
    participant So as kinematics.set_step_mode in kinematics.so
    participant Queue as new_moving_generator in step_controller.py

    alt gait is 5
        Mode->>Follow: construct, then gen_advance_to_first_yield
        Mode->>Queue: store it
        Note over So: not called
    else gait is 1, 2, 3, and the rest
        Mode->>Move: construct, then gen_advance_to_first_yield
        Mode->>Queue: store it
        Note over Move,So: set_step_mode runs later, on the first gen_next_mini_step_yield, when the six parts are baked
    end
```

A newer `Twist` every 50 ms replaces the object sitting in the
queue. The film already playing keeps its pages. The swap waits
for `last_part`.

## 2. One 20 ms tick chooses one walker

`loop()` wakes, takes one page, then calls exactly one of the two
ticks. Legacy is `MovingGenerator` and `CmdVelGenerator`. Hunter
is gait 5, `FollowGaitGenerator`.

```mermaid
sequenceDiagram
    autonumber
    participant Legs as leg thread in step_controller.py
    participant Pull as _pull_mini_step_input() in step_controller.py
    participant State as after_accepted_gait_step() in step_controller.py
    participant Legacy as _tick_legacy_gait() in step_controller.py
    participant Hunter as _tick_hunter_gait() in step_controller.py
    participant Base as set_pose_base() in step_controller.py
    participant So as set_leg_position in kinematics.so
    participant Servo as JointControl in x_joint_control.py

    Note over Legs: time.sleep(0.02) ended. This is one leg tick.
    Legs->>Pull: current stance, generator_slice
    alt a gait-5 page is still open
        Pull-->>Legs: the same six feet, same_target_as_last_wake, hunter
        Note over Pull: gen_next_mini_step_yield is not called. The bookmark stays.
    else a generator is playing
        Pull->>Pull: gen_next_mini_step_yield(stance, generator_slice)
        Note over Pull: The generator yields the next baked page and moves its bookmark.
        alt last_part and the page is legacy
            Pull->>State: counted walk swaps now; velocity walk only orders the outro
        else last_part and the page is gait 5
            Note over State: left for the hunter tick, after the gate accepts the page
        end
    else nothing is playing
        Pull-->>Legs: no feet
        Legs->>Legs: published twist becomes 0, 0, 0
    end

    alt no feet
        Note over Legacy,Hunter: neither walker runs
    else hunter, gait 5
        Legs->>Hunter: frame, stance, walk_state
        Note over Hunter: Diagram 4. May publish eighteen radians, or nothing.
    else legacy
        Legs->>Legacy: frame, walk_state
        alt generator_slice is generator_intro and ask_for_50_ms_blend is true
            Legacy->>Base: six feet, duration 0.05 s
        else every other legacy frame
            Legacy->>Base: six feet, duration 0.02 s
        end
        Base->>So: set_leg_position, once per leg
        So-->>Base: 18 radians
        Base->>Servo: set_multi_joints, those radians, that duration
        Note over Legacy: CmdVelParams adds a full 0.02 s of the command to odometry. A counted walk does not.
    end
    Legs->>Legs: time.sleep(0.02)
```

Legacy `last_part` is handled inside the pull, before the servos
of this page are commanded. The page still plays. The next book
is what starts on a later tick.

The leg thread keeps one `_WalkState` for its whole life. Three
memories live on it. `generator_slice` is the text handed to the
generator on the next resume: `generator_intro` reads the opening
of the prepared list, `generator_running` reads the whole gait
step, and `generator_outro` reads the ending. `generator_outro_order`
says how far an order to play that ending has got. It starts
`'off'`. The wake that notices a velocity walk should change
generators writes `'ordered'`, copies the waiting generator into
`pending_generator_after_outro`, and still sends the feet it
already had, which belong to the slice that just ended. At the
end of that same wake the order becomes `'in_effect'`. The wakes
after that play the outro. The next accepted gait step is the
one that starts the stored generator and sets the slice back to
`generator_intro`. A counted walk does not use this order. It
installs the new generator inside the call.

Gait 5 stores `last_part` on `Gait5Session` and acts on it only
when `approach_target` has accepted the generator's own feet
(`alpha` 1). A half-played page does not change books.

## 3. Where the targets come from

The generator builds the whole gait step in one burst, then yields
one target per `gen_next_mini_step_yield()`. The 20 ms loop plays one mini-step. A
normal mini-step does not recompute the curves.

### Hunter gait 5

For period 0.60 s the bake writes 30 targets. Target `i` is the six
feet at phase `i / 30 * 2π`. `sample_follow_gait` places each toe
on its stance line or on its swing arch. No closed solver runs
during the bake.

```mermaid
sequenceDiagram
    autonumber
    participant Legs as leg thread in step_controller.py
    participant Film as FollowGaitGenerator in move.py
    participant Sample as sample_follow_gait() in move.py

    Legs->>Film: first gen_next_mini_step_yield(stance, generator_intro) after gen_advance_to_first_yield
    Note over Film: Stance object changed, or this is the first bake. The for-loop below does not yield.
    loop one phase per 20 ms of the period
        Film->>Sample: phase, vx, vy, wz, height, period, six homes
        Sample-->>Film: six feet, millimetres
    end
    Note over Film: prepared_targets is the whole cycle. targets_for_generator_intro is the opening. targets_for_generator_outro is the ending.
    Film-->>Legs: yield the first opening frame, last_part, params, ask_for_50_ms_blend
    Note over Legs,Film: About 20 ms later
    Legs->>Film: gen_next_mini_step_yield(stance, generator_slice)
    Film-->>Legs: yield the next stored frame
    Note over Sample: not called. The table already exists.
```

### Legacy films

`CmdVelGenerator` uses the same intro / running / wind-down cut.
The pages come from `kinematics.cmd_vel_new_point` in the closed
library, one call per phase, during the bake.

`MovingGenerator` is the discrete step. One step is 6 parts. Each
part is `period / 6 / 0.02` servo frames, at least one. A 1.00 s
step is 9 frames per part, 54 yields. Each part is one call to
`kinematics.set_step_mode`. The yield's fourth value,
`ask_for_50_ms_blend`, is false, so
the legacy tick always uses the 20 ms blend, and a wrapped step
swaps at once.

```mermaid
sequenceDiagram
    autonumber
    participant Legs as leg thread in step_controller.py
    participant Move as MovingGenerator in move.py
    participant So as kinematics.set_step_mode in kinematics.so

    Legs->>Move: gen_next_mini_step_yield(stance, generator_slice)
    Note over Move: New stance object. Bake six parts before the next yield.
    loop part i from 0 to 5
        Move->>So: sub_action_num, part i, start pose, gait, stride, height
        So-->>Move: several frames of six feet
    end
    Move-->>Legs: yield frame 0 of part 0
    Note over Legs,Move: Each later send yields the next stored frame. last_part is true only when part and sub-frame both wrap to 0.
```

## 4. The hunter tick and the gate

`_tick_hunter_gait` always goes through `Gait5Session` and
`approach_target`. The flag `use_hexapod_kinematics` chooses who
turns the six feet into angles. It is read once, when that
generator's session is created.

- Flag false, the default. `VendorLegIk` calls
  `kinematics.set_leg_position` six times and always allows the
  page. There is no rate gate, so there is no partial page.
- Flag true. `HexapodLegIk` calls `solve_pose` then `gate`. A page
  that is legal but too fast for one 20 ms tick is shortened.
  Any other refusal publishes nothing.

The gate is one yes or no for all eighteen joints. It lives in
`safety.cpp`. It does not move a foot. `approach_target` is the
repair, and it lives in `leg_ik.py`.

A shorter point moves all six feet by the same fraction
along the straight line from the feet where this target started
to the generator's own feet:

```text
feet(fraction) = feet_where_this_target_started
    + fraction * (target_feet_positions - feet_where_this_target_started)
```

Fraction 1 is the page the generator yielded. A smaller fraction
is the same page, not finished. All eighteen angles of that
shorter pose are published together. The CSV column for that
fraction is still named `alpha`.

```mermaid
sequenceDiagram
    autonumber
    participant Tick as _tick_hunter_gait() in step_controller.py
    participant Session as Gait5Session in leg_ik.py
    participant Target as approach_target() in leg_ik.py
    participant Hex as HexapodLegIk.solve in leg_ik.py
    participant Bind as bindings.cpp
    participant Solve as solve_pose() in leg_ik.cpp
    participant Gate as gate() in safety.cpp
    participant Servo as _publish_leg_radians() in step_controller.py

    Note over Tick,Gate: This picture is use_hexapod_kinematics true. Flag false uses VendorLegIk in leg_ik.py and never calls gate().
    Tick->>Session: begin_target on a fresh page, or keep the open one
    Tick->>Session: mini_step()
    Session->>Target: feet_where_this_target_started, target_feet_positions, fraction_of_target_sent
    Target->>Hex: solve(generator's own six feet)
    Hex->>Bind: solve_pose(feet)
    Bind->>Solve: six Vec3, max_step_rad()
    Solve-->>Bind: eighteen radians, or a refusal
    Bind->>Gate: those angles, plus the last angles this gate allowed
    Gate-->>Hex: allow, or rate_limit, or another reason
    alt gate allows the generator's own feet
        Target-->>Session: generators_own_feet_sent, fraction 1, publish
        Session-->>Tick: gait_step_accepted if this page was last_part and the gate sent it
        Tick->>Servo: eighteen radians, duration 0.02 s or 0.05 s
        Note over Tick: Odometry adds fraction_sent_this_wake of 0.02 s. The next tick may call gen_next_mini_step_yield(). The CSV writes decision=sent.
    else reason is rate_limit
        loop up to eight bisections, one shared fraction
            Target->>Hex: solve(feet at that fraction)
            Hex->>Gate: eighteen angles against the last sent angles
        end
        alt some fraction passes
            Target-->>Session: shorter_point_sent, publish those feet
            Tick->>Servo: eighteen radians of the shorter pose
            Note over Session: fraction_of_target_sent stays below 1. Next tick keeps this page. The CSV writes decision=partial.
        else no shorter pose passes
            Target-->>Session: last_safe_pose_kept, publish nothing
        end
    else unreachable, singular, window, or no reference
        Target-->>Session: last_safe_pose_kept, publish nothing
        Note over Servo: The same page would fail again. A later tick does not change that.
    end
```

### Two ticks of one page that was too fast

Suppose the generator's page wants one femur to turn 0.40 rad, and
`max_step_rad()` is about 0.264 rad. The destination feet are
legal. One 20 ms jump is not.

```mermaid
sequenceDiagram
    autonumber
    participant Legs as leg thread in step_controller.py
    participant Pull as _pull_mini_step_input() in step_controller.py
    participant Film as FollowGaitGenerator in move.py
    participant Gate as gate() in safety.cpp
    participant Servo as JointControl in x_joint_control.py

    Note over Legs: Tick A, 20 ms
    Legs->>Pull: target_still_open() is false
    Pull->>Film: gen_next_mini_step_yield()
    Film-->>Pull: this page, bookmark moves
    Pull-->>Legs: six feet, hunter
    Legs->>Gate: angles of the whole page, against the last sent angles
    Gate-->>Legs: rate_limit, the jump is 0.40 rad
    Note over Legs: Bisect in foot space. All six feet move the same fraction.
    Legs->>Gate: angles of the halfway feet
    Gate-->>Legs: allow, leftover turn is about 0.20 rad
    Legs->>Servo: eighteen angles of the halfway pose, duration 0.02 s
    Note over Film: Bookmark stays. gen_next_mini_step_yield() will not be called next tick.

    Note over Legs: Tick B, 20 ms later
    Legs->>Pull: target_still_open() is true
    Pull-->>Legs: the same six feet
    Note over Film: not woken
    Legs->>Gate: angles of the whole page, against the halfway angles
    Gate-->>Legs: allow, the remainder fits
    Legs->>Servo: eighteen angles of the generator's own feet
    Note over Legs: fraction_of_target_sent is 1. If the page was last_part, the book may change now.
```

The second tick cures the refusal because the feet already moved.
The remainder is a smaller turn between the same two legal poses.
A page that is unreachable is a hold: nothing is published, and
waking again on those same feet gets the same refusal.

## 5. The other 50 ms, the servo blend

On the first frame after two films are spliced, the new page can
sit more than 7 mm away from the old feet. The generator then
yields `ask_for_50_ms_blend` true. The leg thread still sleeps 20 ms.
The servo message for that one frame asks for a 50 ms blend.
The next wake, 20 ms later, sends a new target and replaces it.

`MovingGenerator` yields `ask_for_50_ms_blend` false, so this branch never
runs for a discrete step. Its blend is 20 ms.

```mermaid
sequenceDiagram
    autonumber
    participant Film as CmdVelGenerator or FollowGaitGenerator in move.py
    participant Tick as legacy tick or hunter tick in step_controller.py
    participant Servo as JointControl in x_joint_control.py
    participant Legs as leg thread in step_controller.py

    Film-->>Tick: first frame, ask_for_50_ms_blend true
    Tick->>Servo: eighteen targets, duration 0.05 s
    Legs->>Legs: sleep 0.02 s
    Note over Servo: The 50 ms blend is still in progress.
    Legs->>Film: gen_next_mini_step_yield(), or for gait 5 reuse the open page
    Film-->>Tick: the next command
    Tick->>Servo: new targets, usually duration 0.02 s
    Note over Servo: This message replaces the 50 ms one.
```

## Recap

- The hunt publishes a `Twist` every 50 ms. The leg thread plays one baked frame every 20 ms.
- Legacy pages go through `set_pose_base` and `kinematics.so`. Hunter pages go through `Gait5Session`. The rate gate runs only when `use_hexapod_kinematics` is true.
- A partial hunter tick moves all six feet the same fraction of one page. The next 20 ms tick continues that page. The generator is asked for a new page only after `alpha` reaches 1.
