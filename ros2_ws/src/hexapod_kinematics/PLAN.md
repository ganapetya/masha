# Open-source leg IK for hunter gait 15

Package name: `hexapod_kinematics`.

## In plain language

A leg is three hinges. The body says "I want this foot to be here." Something has to answer "then these three hinges must sit at these angles." That answer is inverse kinematics. On Masha, for every other walk, a closed binary (`kinematics.so`) still gives the answer. For hunter gait 15 we will write the answer ourselves, in small pieces, so you can read each piece and see the geometry.

The walking pattern (which foot is in the air, how the step curves) is already ours. This module starts at the moment a foot position exists and ends at three angles. It does not decide where to step.

Picture one leg. The first hinge, the coxa, only turns left and right, like a turret. Once it has turned to face the foot, the other two hinges (femur and tibia) live in a flat triangle, like a two-panel folding ruler. The whole 3-hinge question splits because of that shape: first "which way do I face?", then "how do I fold the ruler so the tip lands on the foot?"

You can check the answer without switching the robot on. Fold the ruler from the angles you just computed (that direction is forward kinematics). The tip should land back on the foot you started with. If it does, the two maps are inverses of each other. That check is the heart of the tests.

The new answer is behind a switch. With the switch off, hunter gait 15 keeps asking the closed binary, exactly as it does today. With the switch on, the same gait asks our module instead. Either way, every pose that gait tries to send is written down, and our module is not allowed to hand the servos a jump, an impossible foot, or an angle outside the travel the driver already enforces. A refused pose holds the last safe angles. It does not twitch a servo to "try."

## What you can learn from this module

Mathematical subjects, in the order the code meets them:

1. **Coordinate frames.** A foot is three numbers only after you have chosen an origin and three directions. Here the origin is the body, +X is toward the head, +Y is to the robot's left, +Z is up. The same physical point, written from a hip, is a different triple. Week 1's frame idea, used on a leg.

2. **Plane trigonometry, including `atan2`.** Sine and cosine turn an angle into the two sides of a right triangle. `atan2` turns two sides back into an angle and keeps track of the quadrant, so a foot behind the hip is not confused with a foot in front of it.

3. **A change of reference direction.** The coxa angle is not "the direction from the nose to the foot." It is that direction minus the direction the leg already points when the servo is at zero (45° for the front-left leg, 90° for the middle-left leg, and so on). Subtracting a fixed angle is a change of frame in the plane.

4. **The law of cosines.** Femur, tibia, and the line from the femur hinge to the foot are three sides of a triangle. Given the three lengths, the angle between femur and tibia is determined. This is the knee.

5. **Inverse kinematics and forward kinematics as a pair.** Forward: angles in, foot out. Inverse: foot in, angles out. The round-trip test is the statement that one undoes the other, inside the region where a triangle exists.

6. **Two solutions, and a rule that is not part of the formula.** A triangle can be folded two ways. The book calls them lefty and righty. Both are mathematically legal. Standing Masha uses one of them (femur raised, tibia pointing down). Picking it is a separate function, so the formula stays the formula and the choice stays visible.

7. **A workspace, and what an impossible question looks like.** From the femur hinge the foot must land in a ring: no closer than the difference of the two link lengths, no farther than their sum. Outside that ring the law of cosines asks for a cosine bigger than 1. The code reports "unreachable" instead of inventing an angle. That is the geometric meaning of a domain.

8. **Units, and the meaning of zero.** Millimetres because the rest of the gait speaks millimetres. Radians because the circle's own unit is the radian; degrees appear only as a reading aid. "Zero" on the servo is the center of its travel, which is not automatically "femur horizontal" in the drawing. An offset written in `geometry` is the sentence that says where the drawing's zero sits relative to the servo's zero. It is a definition, not a fudge factor hidden in the formula.

Alongside the math, the C++ shape is there so the subjects stay separate: one file for the frame numbers, one for the turret angle, one for the triangle, one for the reverse map, one that only calls the others in order, and one that only decides whether those angles are safe to send. A failing test names the subject that broke. You can read a file without knowing ROS callbacks. The ROS part is only the last step, where the hunter's 20 ms loop asks this library for angles and the existing pulse code turns angles into servo commands.

The chapter for each subject is in the reading map below. Read that short list beside the matching file. The rest of each book is later study, after this module makes sense on its own.

## Materials this plan follows

Three books, three jobs. All three opened from the copies you linked, and all three are usable as personal reading beside this package. The PDFs stay on your machine. This package cites sections. It does not store the files, and the comments will not paste the books' prose. Page numbers below are the **printed** page numbers in each book's own contents. A PDF viewer counts the cover and the contents as extra pages, so the viewer's page number sits a little later than the printed one. Search the section number if the two disagree.

Nothing here is taken from the closed binary `kinematics.so`. There is no source to take it from.

### 1. On-ramp — Kala, one chapter

Rahul Kala, *Autonomous Mobile Robots: Planning, Navigation and Simulation*, Academic Press / Elsevier, 2024. ISBN 978-0-443-18908-1.

Read **Chapter 1, §1.4 Kinematics**, and stop at the end of §1.4.3. This chapter names the ideas in plain language. It does not derive the triangle we will code.

| Section | What it is for here |
|---|---|
| §1.4.1 Transformations, Figures 1.3–1.6 | A point's three numbers change when you move the origin or turn the axes. That is why a foot in the body frame and the same foot measured from a hip are different triples. |
| §1.4.2 TF tree, Figure 1.7 | Body, hip, and foot are frames chained together. ROS later calls this picture a TF tree. In this module it is only the picture: we do not publish transforms. |
| §1.4.3 Manipulators, Figures 1.8 and 1.9 | Forward kinematics turns joint angles into a tip. Inverse kinematics turns a tip back into angles, and a planar arm can fold two ways (the book draws elbow up and elbow down). Denavit–Hartenberg parameters are named here. We recognize the name and we do not build a DH table for this leg. |

§1.4.4 is a rolling car. It is the wrong machine for a leg. Chapters after Chapter 1 (localization, mapping, graph search, potential fields, swarms) are a different course.

### 2. The formula — Lynch and Park

Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017 (this copy is the 3 May 2017 preprint, ISBN 9781107156302). In the quarter plan this is Book 2. The authors' site, with videos, is [modernrobotics.org](http://modernrobotics.org). The preprint says it is for personal use. Figures may be reused when the citation is the Cambridge book.

The code follows this book. Comments in `planar_leg` use its names (`L1`, `L2`, `α`, `β`, `γ`, lefty, righty) next to our names (femur, tibia, standing branch), so you can hold **Figure 6.1** beside the file.

| Read this | Printed page | What it is for here |
|---|---|---|
| §3.1 Rigid-body motions in the plane | 62 | The coxa. One angle in the horizontal plane, and subtracting the leg's mount yaw, is a change of frame in that plane. |
| §3.2.1 Rotation matrices | 68 | The same idea written as a matrix, once you want it. The coxa file itself uses `atan2`, which is the angle this matrix is built from. |
| §2.1 and §2.2 | 12 and 15 | Degrees of freedom: a rigid body, then a robot made of joints. Useful as "a leg is three hinges." The quarter plan (`quarter-01-week-01-plan.md`) sends you here under the words "what is a coordinate frame." In this copy the frame picture is §3.1, above. |
| §2.5 Task space and workspace | 32 | The set of feet a leg can actually reach. Short. |
| Chapter 4, the opening, and §4.2 | 137 and 152 | Forward kinematics is the map that runs angles → tip. §4.2 is why a URDF (the CAD file) and the IK hips can be different frames. The code takes three geometric steps. It does not use the product-of-exponentials formula that fills the rest of the chapter. |
| Chapter 6 opening, **Figure 6.1**, through the lefty and righty formulas | 219–220 | **This is the formula.** Law of cosines, `atan2`, the ring-shaped workspace, and the two folds. §6.1 starts on printed page 221 with a 6-joint PUMA arm. Stop before §6.1. |

The coxa angle is Figure 6.1's `γ = atan2(...)`, measured from the leg's mount direction. Subtracting that mount yaw is the planar frame change from §3.1.

Lynch's own paragraph on printed page 219 says the two folds are called lefty and righty, and also elbow-up and elbow-down. Jazar uses the second pair of names. Same two triangles.

Chapters 5 and 7 onward (velocity, closed chains, dynamics, trajectories, motion planning, control) wait until this leg is clear.

### 3. The second pass — Jazar

Reza N. Jazar, *Theory of Applied Robotics: Kinematics, Dynamics, and Control*, 2nd edition, Springer, 2010. ISBN 978-1-4419-1749-2.

This is the deeper kinematics textbook of the three. Read it after the matching Lynch section, when you want the same idea said with the classical names. The cooperation memo's `book1.pdf` is still an untitled filename, so this plan does not label Jazar as Book 1. If that file turns out to be this book, the map below is already the citation. If it is a different book, its matching sections get added when the file can be identified.

| Read this | Printed page | What it is for here |
|---|---|---|
| §1.3.2 Workspace | 13 | Reachable workspace: where a tip can go. A cosine outside `[−1, 1]` is a foot outside that set. |
| §1.4.3 Reference frame and coordinate system | 17 | Origin plus axes, in the classical wording. |
| §2.1 Rotation about a global axis | 33 | One yaw, written as a matrix. Enough of Chapter 2 for the coxa. The later catalogue (Euler angles, roll-pitch-yaw) is a different topic. |
| §4.1, §4.2, §4.4 | 149, 154, 168 | A rigid motion, one homogeneous transform, then a chain of them. This is Kala's TF tree written with matrices. |
| §5.1 Denavit–Hartenberg, §5.3 Forward position kinematics | 233, 259 | The classical way to chain a whole arm. Read so you recognize it. This leg stays the planar triangle; the code does not fill in a DH table. |
| Chapter 6, Figure 6.1 and Figure 6.3 | chapter starts 325 | Figure 6.1: a planar 2R has more than one inverse solution. Figure 6.3: elbow up and elbow down. That pair is our lefty and righty. |
| §6.4.1 Existence and uniqueness | 361 | When a solution exists, and when two of them do. |
| §6.5 Singular configuration | 363 | At the boundary of the ring the two folds meet. The code's answer is still the cosine test: outside the ring, refuse. |

Jazar's later parts (dynamics, control) and the iterative inverse-kinematics techniques in §6.4.2 are outside this module. We have a triangle with a closed-form answer.

### Which file to open with which pages

Read the row before you read that file. The files do not exist yet. This is the order they will be written in.

| Future file | Read first |
|---|---|
| `geometry` | Kala §1.4.1–§1.4.2 and Figure 1.7. Lynch §3.1. Jazar §1.4.3. The hip coordinates, the link lengths, and the joint zeros are this robot. No chapter lists them. |
| `coxa` | Lynch §3.1, then the `atan2` paragraph in the Chapter 6 opening (printed page 219). Jazar §2.1 if you want the one-axis matrix behind that subtraction. |
| `planar_leg` | Lynch Chapter 6 opening and Figure 6.1 (printed pages 219–220). Then Jazar Chapter 6, Figures 6.1 and 6.3, so elbow up / elbow down lands on lefty / righty. |
| `forward` | Kala §1.4.3 and Figure 1.8. Lynch Chapter 4 opening: forward kinematics is the opposite map. Jazar §5.3 is the classical chain. The file walks the three links geometrically. |
| `leg_ik` | No new chapter. It calls `coxa`, then `planar_leg`, in that order. |
| `safety` | Lynch §2.5 and Jazar §1.3.2, §6.4.1, and §6.5, for the idea "outside the workspace, do not invent an angle." The ±120° window is `JointControl`'s travel. `max_step_rad` is a measurement on this robot. Neither number is a formula from these books. |

### Numbers that are not in any of the three books

The books have no Masha link lengths and no servo center.

- Hip positions and the 45° / 90° / 135° mount angles: the offsets already named `X1`, `Y1`, `Y2` in `build_in_pose.py`, checked against the coxa angle the robot uses at the stand pose.
- Starting link lengths: distances between joint origins in `rospider_description/urdf/base.urdf.xacro` (coxa about 45 mm, femur about 77 mm, tibia about 116 mm). The round-trip test is allowed to correct them. The comment will say which test fixed the final value.
- What "zero" means to the servos: the three stand angles `set_leg_position` returns for `DEFAULT_POSE`. Those readings calibrate the offset. They are not the algorithm. With the flag on, the gait-15 path does not call `kinematics.so`. With the flag off, it still does.

## What this is

Hunter gait 15 already decides where each foot should be. The closed piece is the next question: given one foot tip in millimetres, what three joint angles put it there. Today that answer comes from `kinematics.so` (`kinematics.set_leg_position`). This task writes that answer ourselves, in C++, and calls it only for gait 15.

The new code is a kinematics module, not a faster walker and not a rewrite of the foot curves. A reader should be able to open one file, read the comment, and see one piece of the math.

## Where it sits

```
masha_hunter_node                         proud_up, unchanged
  Traveling gait 15                       select only; stores cmd_gait = 5
  Twist every 50 ms
        │
        ▼
StepController 20 ms loop                 controller/step_controller.py
  FollowGaitGenerator                     six foot tips, already ours
        │
        ├─ gait == 5 and use_hexapod_kinematics true
        │     → hexapod_kinematics, then the safety gate
        ├─ gait == 5 and the flag false (the default)
        │     → kinematics.set_leg_position, today's path
        └─ any other gait
              → kinematics.set_leg_position, no new gate
        │
        ▼
JointControl                              radians → servo pulses
                                            a refused gait-5 pose never arrives here
```

Every gait-5 pose, from either backend, is one row in the trace file. A refused pose is a row with no pulse.

Gait 15 never called `kinematics.set_step_mode`. The `.so` enters this walk only inside `StepController.set_pose_base` (`step_controller.py` around the line that does `kinematics.set_leg_position(i + 1, position)` for all six feet). The 20 ms loop applies a moving pose around the `set_pose_base(moving_pose, ...)` calls. That is the only insertion point.

`params.gait == 5` is the hunter follow generator (`cmd_gait` was set by Traveling 15, or by the explicit gait-5 door). Joystick gaits, voice gait 2, body lean, and action groups keep the vendor call. Head servos are not in this chain. Halt (`gait` 0, then `gait` -2 / `DEFAULT_POSE`) stays on the vendor path; the stand-pose test below is what keeps that handoff from snapping a leg.

## Package

New ament package `src/hexapod_kinematics`. No `rclcpp`. It is a library of pure functions plus a thin pybind11 module. `pybind11` is already on this Jetson (`pybind11-dev`, `/usr/include/pybind11/pybind11.h`).

```
src/hexapod_kinematics/
  package.xml
  CMakeLists.txt
  include/hexapod_kinematics/
    types.hpp          # Vec3, JointAngles, IkResult, leg ids
    geometry.hpp       # hips, mount yaws, link lengths, joint zeros
    coxa.hpp           # subproblem 1
    planar_leg.hpp     # subproblem 2
    forward.hpp        # the reverse map, for tests and for reading
    leg_ik.hpp         # composition only
    safety.hpp         # finite, reachable, travel window, step size
  src/
    geometry.cpp
    coxa.cpp
    planar_leg.cpp
    forward.cpp
    leg_ik.cpp
    safety.cpp
    bindings.cpp       # no formulas
  test/
    test_coxa.cpp
    test_planar_leg.cpp
    test_forward.cpp
    test_leg_ik.cpp
    test_safety.cpp
```

Namespace `hexapod_kinematics`. One translation unit per subproblem. The header states the contract (inputs, units, what must already be true). The `.cpp` file starts with a comment that names the subproblem in plain language, then the formula. Binding file only converts Python floats to those calls.

`controller/package.xml` gains an `exec_depend` on `hexapod_kinematics`.

## The math, in the order a reader meets it

Body frame, same as `follow_gait.hpp` and `DEFAULT_POSE`: **+X toward the head, +Y to the robot’s left, +Z up**, feet at negative z, **millimetres**. Leg ids 1..6 are LF, LM, LR, RR, RM, RF. Angles we return are **radians in the same sense `set_leg_position` returns today**, before `JointControl` applies per-servo `direction`. We do not compute pulses.

### 1. `geometry` — the constants, and nothing else

Read first: Kala §1.4.1–§1.4.2 (Figure 1.7), Lynch §3.1, Jazar §1.4.3. The numbers in this file are the robot's, not a chapter's.

Three kinds of number, written apart so a length is never hiding inside a formula:

- **Yaw pivot of each leg** (where the coxa servo axis meets the body). These are the points the foot coordinates are actually measured from, the same offsets `build_in_pose.py` names `X1`, `Y1`, `Y2`:

  | leg | name | hip x, y (mm) | mount yaw (coxa = 0) |
  |---|---|---|---|
  | 1 | LF | (+93.60, +50.805) | +45° |
  | 2 | LM | (0, +73.535) | +90° |
  | 3 | LR | (−93.60, +50.805) | +135° |
  | 4 | RR | (−93.60, −50.805) | −135° |
  | 5 | RM | (0, −73.535) | −90° |
  | 6 | RF | (+93.60, −50.805) | −45° |

  Checked against the vendor coxa angle: at `DEFAULT_POSE` the LF foot `(163.6, 140.8, −70)` gives coxa **+6.91°**, and `atan2` from `(93.60, 50.805)` minus 45° lands on that. LM’s home foot gives coxa **0°**, which is a foot straight out from `(0, 73.535)` along +90°. Moving a foot only in z leaves the coxa angle unchanged. The URDF `leg_center` (LF is about `(104.6, 50.6, 16.6)` mm) is the mesh frame, a different point. This module does not solve the CAD chain. A comment in `geometry.cpp` says that, so a reader who opens `base.urdf.xacro` does not think the two hips should match.

- **Link lengths**, shared by all six legs (the hardware is the same; mirror is the mount yaw). Starting guesses from the URDF joint-origin distances, to be confirmed by the round-trip in step 4, not left as unexplained literals:
  - coxa, hip axis to femur axis: **45.0 mm**
  - femur, femur axis to tibia axis: **77.1 mm**
  - tibia, tibia axis to foot tip: **115.6 mm**
  - hip height in the same z as the foot coordinate: start at **0** and correct it if the round-trip needs a constant z shift. The comment records the final value and which test fixed it.

- **Joint zero**. Geometric angles (femur measured from horizontal, tibia measured from the femur) are not automatically the radians the servo driver expects. Three offsets live here. They are filled in when `solve_leg(DEFAULT_POSE)` is required to reproduce the known stand:

  LF stand target, radians: coxa **0.1207**, femur **0.7555**, tibia **−0.650** (6.91°, 43.29°, −37.24°). The other five legs follow by mirror: coxa flips with the foot’s y, femur and tibia stay the same number. Right-side servos flip in `config.SERVOS` `direction`, which is `JointControl`’s job. This module must not add a second sign for the right femur.

### 2. `coxa` — yaw, one angle, horizontal plane only

Read first: Lynch §3.1, and the `atan2` paragraph on printed page 219 of the Chapter 6 opening. Jazar §2.1 is the same yaw written as a matrix.

Input: leg id and foot `(x, y)`. z is ignored.

```
dx = x - hip_x
dy = y - hip_y
q_coxa = atan2(dy, dx) - mount_yaw
```

Wrap into `(−π, π]`. `atan2` takes **(y, x)**, not `(x, y)`. The file comment says so with the LF stand number as the worked example: from the hip the foot sits a few degrees past the 45° mount, and that leftover is the coxa angle.

This file does not know femur, tibia, or link length.

### 3. `planar_leg` — the knee triangle (Figure 6.1)

Read first: Lynch, Chapter 6 opening and Figure 6.1, printed pages 219–220, and stop before §6.1. Then Jazar, Chapter 6, Figures 6.1 and 6.3. Lynch's page 219 says lefty/righty are also called elbow-up and elbow-down, which is Jazar's name for the same two folds. The code keeps Lynch's names.

After the coxa yaw is known, the foot lies in a vertical plane. Subtract the coxa link along that heading. What remains is the planar arm in the book: femur is `L1`, tibia is `L2`, and the target `(x, y)` in the book's drawing is our `(x_plane, z_plane)`.

```
x_plane = hypot(dx, dy) - coxa_length
z_plane = z_foot - hip_z
```

`x_plane` is the horizontal distance from the **femur axis** to the foot. `z_plane` is the book's `y`: vertical, negative when the foot is below the hip. A comment on those two lines says this renaming, so the drawing and the robot frame are not mixed up.

Write Figure 6.1 out, with the book's symbols, then name our joints underneath:

```
r² = x_plane² + z_plane²
cos β = (L1² + L2² − r²) / (2 · L1 · L2)     # interior angle opposite the line to the foot
cos α = (r² + L1² − L2²) / (2 · L1 · r)
γ = atan2(z_plane, x_plane)
```

If either cosine is outside `[−1, 1]`, the foot is outside the ring in Figure 6.1(a): inner radius `|L1 − L2|`, outer radius `L1 + L2`, measured from the femur axis. Return `ok = false` and the reason `"unreachable"`. Do not invent a clamped angle. That is the book's workspace, and it is the whole point of the check.

The book then gives both solutions (Figure 6.1):

```
righty:  θ1 = γ − α,   θ2 = π − β
lefty:   θ1 = γ + α,   θ2 = β − π
```

`θ1` is the femur angle in the drawing, `θ2` is the tibia angle relative to the femur. Both are computed. `standing_branch` keeps the one that matches a standing Masha: femur positive, tibia negative at `DEFAULT_POSE`. The comment records whether that turned out to be lefty or righty, after the stand numbers decide it. Raising the foot (z from −70 toward −50) must increase the femur angle and make the tibia more negative. The other solution stays in the test, so you can see the triangle flip, and the walker does not use it.

Then add the joint-zero offsets from `geometry`. Those offsets are not in Figure 6.1. The book's zero is "however the drawing defines θ = 0." The servo's zero is the center of its travel. The law-of-cosines block does not mention servos.

### 4. `forward` — angles back to a foot

Read first: Kala §1.4.3 and Figure 1.8, then the opening of Lynch Chapter 4. Jazar §5.3 is the classical chain. This file walks three links. It does not build the product of exponentials or a Denavit–Hartenberg table.

Same constants, opposite direction:

1. Start at the hip.
2. Step `coxa_length` along `mount_yaw + q_coxa`.
3. Step the femur, then the tibia, in that vertical plane, using the geometric angles (joint zeros removed first).

Output is a foot in the body frame, millimetres. Tests use this. Runtime gait does not.

### 5. `leg_ik` — composition only

Read first: no new chapter. This file calls `coxa`, then `planar_leg`.

`solve_leg(leg_id, foot) -> IkResult`:

1. Read that leg’s hip and mount yaw.
2. `solve_coxa`.
3. Build `(x_plane, z_plane)`.
4. `solve_planar`.
5. Apply joint zeros.
6. Return `{ok, coxa, femur, tibia}` or `{ok: false, reason}`.

No new formula in this file. A six-leg helper `solve_pose` loops legs 1..6 so the Python side can do one call per 20 ms tick. If any leg fails, the whole pose fails, with the leg id and the reason (`unreachable`, or a non-finite number). The Python binding returns that result. It does not raise. Raising would fall into the loop's existing `MOVING_POSE` handler, which drops the generator. The hunter's next Twist, 50 ms later, would build a new generator and try the same bad pose again. The plug below holds the last safe angles instead.

### 6. `safety` — what is allowed to reach a servo

Read first: Lynch §2.5 and Jazar §1.3.2, §6.4.1, and §6.5, for "a foot outside the reachable set has no angle." The ±120° window and `max_step_rad` are this robot's driver and a measurement. They are not a formula in these books.

A separate file, `safety.hpp` / `safety.cpp`. It does not solve a triangle. It answers one question: given the angles we want now and the angles we last actually sent, may this pose be published?

Checked in this order, all 18 joints, and the pose is one decision:

1. **Finite.** Any NaN or infinity → refuse. Reason `non_finite`.
2. **The solver agreed.** `solve_pose` returned `ok == false` → refuse. Reason kept from the solver (`unreachable`).
3. **Travel window, the same one the driver already uses.** `JointControl` rejects a joint when `|direction * (offset + radians)|` is greater than half of `max_radians`. In `config.py` that half-window is **±120°** (±2.094 rad), because the servo travels 240° around the center pulse 500. The gate applies that test to every joint **before** `set_multi_joints` is called. One joint outside the window refuses the whole pose. Reason `joint_limit`, and the log names the joint. The numbers are read from the same `SERVOS` table the driver uses. They are not a second, quieter limit.
4. **Step size.** `|q_now − q_previous|` for every joint must be ≤ `max_step_rad`. This is the check that stops a lefty/righty flip: that flip moves a knee by about a radian in one 20 ms tick, which is a kick. A real hunter step moves a joint by only a few degrees per tick. `max_step_rad` is **measured, then written down**, not chosen to look round. The calibration script runs one gait-5 cycle with the vendor backend, no servos (`pseudo`), at the hunter's speeds (the Twist clamps: 0.12 m/s, 0.10 m/s, 0.6 rad/s, period 0.60 s) and records the largest `|Δq|` in one tick. `max_step_rad` is that peak times **1.5**. The comment in `safety.cpp` states the measured peak, the margin, and the command that produced them. A pose that fails is reason `rate_limit`, and the log names the joint and both angles.

`q_previous` is the last pose this gate **sent**. `joints_state` is initialized to 0 in `StepController.__init__`, which is not where the servos actually are, so it is not the reference. `self.pose` starts as `DEFAULT_POSE`, and a hunt starts from that stand. The first sample of a bout is compared with `solve` of the current `self.pose` through the same backend. If that reference itself cannot be solved, send nothing and log `no_reference`.

The gate returns `{allow, reason, joint_id}`. It never clamps an angle to the limit. A clamped angle would move the foot off the spot the gait asked for and hide the failure.

## The plug

Gait 5 is the only caller. The two backends sit behind one small Python object, `controller/leg_ik.py`, so the 20 ms loop does not contain two copies of the pulse code.

```python
class LegIk:
    def solve(self, pose) -> IkDecision: ...

class VendorLegIk(LegIk):      # kinematics.set_leg_position, one leg at a time
class HexapodLegIk(LegIk):     # hexapod_kinematics.solve_pose, then safety.gate

def make_leg_ik(use_hexapod: bool) -> LegIk: ...
```

`IkDecision` is `allow`, the 18 radians if allowed, and a reason string either way. `VendorLegIk` does not run the new rate gate. Its angles are the reference the rate limit was measured from, and `JointControl` still enforces the ±120° window on them as it does today. `HexapodLegIk` runs the gate. On `allow == false` it returns the **previous sent** radians and `allow` stays false, so the caller publishes nothing.

The flag is a ROS parameter on the step_controller node:

- name: `use_hexapod_kinematics`
- type: bool
- **default: false**

Declared in `StepController.__init__`, so `ros2 param set /step_controller use_hexapod_kinematics true` works and the default is visible with `ros2 param get`. Also a launch argument of the same name on `move_controller.launch.py`, default `false`, passed into the node. The hunter launch does not own this parameter: hunter and the step controller are different processes, and the controller is the process that calls IK.

The flag is read when a gait-5 generator **starts**, and that generator keeps one backend until it is replaced. A param change mid-step does not swap formulas under a moving foot. The next generator (the next Twist after a step boundary, or the next hunt bout) picks up the new value. A comment says that.

Joystick gaits, voice gait 2, body lean, and action groups never call `make_leg_ik`. They stay on `set_pose_base` and `kinematics.set_leg_position`.

## Logging

Two sinks. The screen stays readable. The file has every move.

**Trace file**, one row per gait-5 pose, both backends. Path parameter `hexapod_ik_trace`, default `/home/ubuntu/ros2_ws/log/hexapod_ik_trace.csv`. Append. Write the header when the file is created.

Columns:

- `t_s` — controller clock, seconds
- `backend` — `hexapod` or `vendor`
- `decision` — `sent`, `held`, or `rejected`
- `reason` — empty when sent; otherwise `unreachable`, `joint_limit`, `rate_limit`, `non_finite`, `no_reference`
- `joint` — the joint id that failed, or empty
- six feet, millimetres: `x1 y1 z1` … `x6 y6 z6`
- eighteen angles, radians: `q1` … `q18` in kinematic servo order (coxa, femur, tibia, leg 1 then leg 2 …)
- eighteen steps from the previous **sent** pose, radians: `dq1` … `dq18`

`held` means the gate refused and no pulse went out; the `q` columns are the angles that were refused, so a later plot shows the command that was blocked. A streak is still one row per tick. Nothing is sampled out. A 0.60 s step at 20 ms is 30 rows.

**ROS log**, on the step_controller logger:

- Once, at node start: the flag, the trace path, the joint window (±120°), and `max_step_rad` with the measured peak beside it.
- Once per gait-5 generator: `gait 5 backend=hexapod` or `backend=vendor`.
- Every refusal: `WARN`, with reason, joint, the angle, and the foot of that leg. These are the lines you watch while standing next to the robot.
- Five refusals in a row (100 ms): one `ERROR`, `holding last safe pose`. The generator is **not** dropped. Publishing stays off until a later tick passes the gate, or until a stand (`gait` -2) replaces the motion. The streak counter resets when a pose is sent.
- A 1 Hz `INFO`: backend, last decision, and the largest `|dq|` in that second. The 50 Hz detail stays in the CSV.

`pseudo=True` still solves, still gates, still writes a row with `decision=rejected` or `sent`, and still does not publish. A dry run is a trace with no motion.

## Wiring in the 20 ms loop

`set_pose_base` stays the vendor path for every non-gait-5 caller.

At both places that send `moving_pose` to the servos (the 20 ms tick and the 50 ms first-slice blend): if `params` is a `CmdVelParams` and `params.gait == 5`, call the plug selected for this generator.

- `allow`: `JointControl.set_multi_joints` with those radians, same ids 1..18 as today. Then remember them as `q_previous`.
- `allow` false: publish nothing. Do not update `joints_state`. Write the trace row. Count the streak.

A comment above the branch: gait 5 is the hunter follow walk; `use_hexapod_kinematics` chooses who turns foot tips into angles; a refusal sends no pulses.

`follow_gait.hpp` and `FollowGaitGenerator` stay the source of the foot tips. Their comments that say the vendor function turns tips into angles get one sentence: with the flag on, `hexapod_kinematics` does that job, and the safety gate can refuse the result.

## Comments a student should find

- File top: which subproblem, in one or two sentences, and what this file deliberately does not know.
- Function: units, when it runs (gtest, or the 20 ms loop via pybind), what must already be true.
- The `atan2` argument order, the wrap, and the LF stand numbers as a worked line.
- Why a failed triangle returns `unreachable` instead of a quiet clamp.
- Why joint zero is not inside the law of cosines.
- Why pulses and `SERVOS['direction']` stay in Python.
- Why a refusal publishes nothing, and why the generator is not dropped.
- Why `max_step_rad` has a measured peak written next to it.

No comment that only repeats the syntax.

## Tests (gtest, no robot, no `.so`)

Each test file matches one subproblem, so a failure names the piece.

- **Coxa.** Foot on the mount ray from the LF hip → coxa 0. LF stand foot → coxa within 0.01 rad of +0.1207. Same foot with a different z → same coxa. LM home foot → coxa 0. Leg 6 is the mirror of leg 1.
- **Planar.** One hand-built triangle from Figure 6.1 (pick `x_plane`, `z_plane`, compute `α`, `β`, `γ` on paper) → solver returns those angles. A point with `r > L1 + L2` → `ok == false`. Lefty and righty are both visible; `standing_branch` is the one with tibia negative when the foot is below the hip.
- **Forward round trip.** For a grid around each `DEFAULT_POSE` foot (about ±40 mm in x and y, z from −100 to −40, which covers the hunter lift), `forward(solve_leg(p))` is within 0.5 mm of `p`. Skip points the solver marks unreachable.
- **Stand contract.** `solve_leg` on all six `DEFAULT_POSE` feet matches the vendor stand radians within **0.02 rad** on each joint. Those target numbers are written in the test with the degree values beside them. This is the gate that says our zero is the servo’s zero.
- **Safety.** A joint at 2.2 rad (past ±120°) → `joint_limit`, and the decision would publish nothing. A 1 rad jump from the previous sent pose → `rate_limit`. A step of a few hundredths of a radian inside the window → `allow`. A NaN → `non_finite`. The test does not call a servo.

A short Python script under `hexapod_kinematics` (not imported by the controller) prints our angles next to `kinematics.set_leg_position` for the stand and a few offsets, and prints the peak `|Δq|` over one pseudo gait-5 cycle. That peak, times 1.5, is what gets written into `safety.cpp`. With the flag false, a live hunt still uses `kinematics.so`. With the flag true, the gait-5 path does not.

## Build and the live check

1. `colcon build --packages-select hexapod_kinematics` and the four gtests.
2. Build `controller` after the pybind module installs, import `hexapod_kinematics` from the sourced workspace.
3. Run the calibration print. If any stand joint is outside 0.02 rad, fix lengths or joint zeros and re-run the gtests. Write the measured `max_step_rad` into `safety.cpp` and re-run `test_safety`. Do not turn the flag on while either gate fails.
4. A hunt with the flag left false is the before-picture: the trace file fills with `backend=vendor`, and the walk is the one you already know.
5. Live with the flag true, only after that and only when Peter asks: `ros2 param set /step_controller use_hexapod_kinematics true`, then start the hunter and let gait 15 walk a short follow, then halt. Halt still uses the vendor stand. Watch the WARN lines and the first rows of the trace. A `rate_limit` or `joint_limit` must leave the legs where they were. Joystick walk is unchanged and is the regression check that gait 2 still calls the `.so`.

## Work order

1. **This commit.** The `hexapod_kinematics` package exists so this plan has a home (`PLAN.md`). No solver, no plug, no flag wiring in this commit. Commit the package before any of the math below is written.
2. Library skeleton: `types`, `geometry` with the hip table and the length guesses.
3. `coxa` and `test_coxa`.
4. `planar_leg` and `test_planar_leg`, including the unreachable case and the standing branch.
5. `forward`, round trip, then adjust lengths and joint zeros until the stand contract passes. Write the final numbers into `geometry.cpp` with a comment that names the test.
6. `leg_ik` composition and `solve_pose`. The binding returns the result and does not raise.
7. `safety` and `test_safety`. `max_step_rad` stays a named constant with a comment; the calibration script fills the number before the flag may be turned on.
8. pybind11 module, including the safety gate.
9. `controller/leg_ik.py`: the two backends, the flag defaulting to false, the trace row on every gait-5 pose, WARN on each refusal, no publish when the gate says no.
10. The `params.gait == 5` branch in the 20 ms loop, and the launch argument on `move_controller.launch.py`.
11. Calibration print (stand error and the measured step peak), then stop for a live hunt. First hunt keeps the flag false. The flag-true hunt waits until you ask.
