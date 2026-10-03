# Learn Masha's kinematics from the code

This is a study path through the library that walked on 3 October 2026.
Read one sitting, then open the file it names, then run or re-read the
test it names. The design history and the book pages live in
[PLAN.md](PLAN.md). This file is the order to learn the code that is
already written.

A sitting is about an hour. Stop at the recap. If you cannot say the
recap without looking, stay on that sitting.

## What the hunt actually ran

The hunt that finished at 14:58 used this library for every gait-5
step. The evidence is in one controller log and one trace.

| What | Where | What it says |
|---|---|---|
| Controller log | `~/.ros/log/python3_2060_1791018908627.log` | Process started 12:15:08. Walk from 14:43:17 to 14:58:36. |
| Startup line | same file, first line | `use_hexapod_kinematics=False` at process start. Also `max_step_rad=0.26433175300161293`, which is the constant in `safety.cpp`. |
| Generator lines | same file | `gait 5 backend=hexapod` 224 times. `backend=vendor` zero times. |
| One-hertz lines | same file | 157 lines `backend=hexapod`. 156 of them `decision=sent`, one `decision=partial`. No ERROR. |
| Trace | `~/ros2_ws/log/hexapod_ik_trace.csv` | 7221 rows, every row `backend=hexapod`. 7096 `sent`, 125 `partial`, zero `held`. Every partial reason is `rate_limit`. |
| Built library | `install/hexapod_kinematics/` | `libhexapod_kinematics.so` and `hexapod_kinematics.cpython-310-aarch64-linux-gnu.so`, both built 11:30, before the walk. |

The startup line and the walk lines answer different moments. The node
is created with the parameter false. `_begin_gait5` in
`step_controller.py` reads the parameter again when a gait-5 generator
starts, and `begin_generator` prints `gait 5 backend=hexapod` only when
that read is true. That call constructs `HexapodLegIk`, whose `solve`
is `hexapod_kinematics.solve_pose` followed by `hexapod_kinematics.gate`.

Two more facts that only the new path can produce:

- A `partial` row exists only in `approach_frame`, and only after the
  gate returns `rate_limit`. `VendorLegIk` always returns allow, and it
  has no bisection.
- The partial alphas are on the 1/256 grid (`0.4609375` is 118/256,
  `0.99609375` is 255/256). That grid is `BISECTION_PARTS` in
  `leg_ik.py`.
- The largest `|dq|` stored on a sent row is `0.26431329` rad. The
  gate's limit is `0.26433175300161293` rad. The sent steps sit under
  that limit.
- All 125 partials named a femur: joint 2 (LF) 85 times, joint 5 (LM)
  28, joint 14 (RM) 6, joint 17 (RF) 3, joint 11 (RR) 2, joint 8 (LR) 1.
  The calibration note in `safety.cpp` says the measured peak is a femur.

7221 rows is about 144 seconds of 20 ms gait ticks, spread through a
hunt of about 15.6 minutes. The other minutes were not gait-5 solves.

The controller process that is up after that hunt was started later.
Its launch parameters set `use_hexapod_kinematics` to false, and
`ros2 param get /step_controller use_hexapod_kinematics` returns false.
A new hunt on that process uses `VendorLegIk` until the parameter is
set true again. Both `.so` files can be mapped in that process anyway:
`step_controller.py` imports `kinematics.so` at import, and
`read_max_step_rad()` imports this package at startup so it can print
the limit. A mapped file is the library being present. `backend=hexapod`
is the library being called.

## The picture to keep

One gait-5 tick, when the flag is true:

```
masha_hunter_node
  Traveling, gait 15, which stores cmd_gait = 5
  Twist on /controller/cmd_vel
        |
        v
FollowGaitGenerator          move.py
  six foot tips, millimetres, body frame
        |
        v
StepController.loop          a Python thread, sleep 0.02 s
  Gait5Session.step
        |
        v
HexapodLegIk.solve
  hexapod_kinematics.solve_pose     C++, six legs
  hexapod_kinematics.gate           C++, one yes or no
        |
        +-- allow: JointControl turns 18 radians into pulses
        +-- rate_limit only: walk part of the way, publish that, stay on this frame
        +-- any other refusal: publish nothing, stay on this frame
```

This package does not choose the step. The generator already chose the
six tips. This package answers: which eighteen radians put the feet
there, and may those radians be sent.

`kinematics.so` still answers every gait that is not gait 5, and it
answers gait 5 when the flag is false. A halt to the stand still goes
through `set_pose_base`, which calls `kinematics.set_leg_position`.

There is no ROS node inside this package. No subscription, no timer,
no TF broadcast. The C++ is a function you call. The ROS 2 piece is
the parameter on `/step_controller` and the thread that calls the function.

### Recap

- The generator owns the feet. This library owns the angles and the yes-or-no.
- Gait 5 with the flag true calls `solve_pose` then `gate`.
- Pulses are `JointControl`, after a yes.

## Words, before the files

Learn these once. Later sittings use them without redefining them.

**Body frame.** Origin in the body. +X toward the head, +Y to the
robot's left, +Z up. Millimetres. Feet stand at negative z. This is
the same frame as `DEFAULT_POSE` and the follow gait. It is a C++
struct `Vec3`, not a ROS message, and it has no `frame_id`.

**Leg numbers.** 1 LF, 2 LM, 3 LR, 4 RR, 5 RM, 6 RF. Arrays in this
package are indexed 0..5. `leg_index` is the only subtraction.

**Coxa, femur, tibia.** The three hinges of one leg. Coxa yaws in the
horizontal plane, like a turret. Femur and tibia fold in the vertical
plane the coxa is facing, like a two-panel ruler.

**Inverse kinematics.** Foot in, angles out.

**Forward kinematics.** Angles in, foot out. Here it is the check that
the inverse can be undone.

**Joint zero.** The drawing measures the femur from horizontal and the
tibia from straight. The servo measures both from the centre of its
travel. The zero is the gap between those two rulers. It is added
after the triangle exists.

**Servo radian vs pulse.** This library returns radians in the sense
`set_leg_position` returns them. `JointControl` applies
`SERVOS['direction']` and turns the radian into a pulse around 500.
Doing that conversion twice would move the foot.

**A refusal.** A returned object with a reason. The functions in this
package do not throw for an impossible foot. An exception in the 20 ms
loop drops the generator. The wrong number of feet is the exception:
that is a caller bug, and the binding raises `ValueError`.

**C++ and ROS 2, side by side.** `enum class`, `struct`, `constexpr`,
`std::array`, and returning a struct by value are C++. A parameter,
a logger line, and a thread that sleeps 20 ms are the Python ROS node.
pybind11 is the door between them.

## The files, one sentence each

| File | The one job |
|---|---|
| `include/.../types.hpp` | The shapes every other file passes around. |
| `src/geometry.cpp` | The hips, the link lengths, the joint zero, the stand tape. |
| `src/coxa.cpp` | The yaw. |
| `src/planar_leg.cpp` | The knee triangle, both folds, and the soft edge. |
| `src/forward.cpp` | Angles walked back to a foot. |
| `src/leg_ik.cpp` | Coxa, then triangle, for one leg, then for six. |
| `src/safety.cpp` | Whether those eighteen angles may be published. |
| `src/bindings.cpp` | Python lists in, the C++ decision out. |
| `CMakeLists.txt` | Two shared libraries and the tests. |
| `controller/leg_ik.py` | Which backend, the catch-up, the session, the trace. |
| `controller/step_controller.py` | The 20 ms thread that calls the session on gait 5. |
| `scripts/calibrate_max_step.py` | The measurement that became `kMaxStepRad`. The controller does not import it. |

`README.md`, the comment at the top of `CMakeLists.txt`, and the
`<description>` in `package.xml` name the same caller.
`StepController.loop` calls this plug when `params.gait == 5` and the
flag is true.

## Sitting 1 — Shapes (`types.hpp`)

### The idea

Before any trigonometry, the code needs names for a point, three
angles, a leg, and a yes-or-no. If those names were rewritten inside
each file, a "radian" in one file could silently be a pulse in another.

### Where it sits

`types.hpp` is included by every later header. Nothing includes ROS.
`gtest` and the Python binding both speak these types.

### What you need

An `enum class` is a C++ enum that does not mix with a bare `int`.
`LegId::Lf` is not the integer 1 until someone casts it. That cast
lives in `leg_index`. A `struct` here is a bundle of numbers with
names, returned by value, so the caller owns the copy.

### The mechanism

`Vec3` is millimetres. `JointAngles` is radians, coxa then femur then
tibia, before `JointControl`.

`IkReason` is the vocabulary of every refusal you will meet:

| Enumerator | The English in the trace | Who is allowed to set it |
|---|---|---|
| `None` | empty string | A solve that may be sent. |
| `Unreachable` | `unreachable` | The triangle, when the cosine is outside `[-1, 1]`. |
| `NearSingular` | `near_singular` | The triangle, when a 0.5 mm foot error would swing the knee too far. |
| `JointLimit` | `joint_limit` | The gate, when a servo would leave the ±120° window. |
| `RateLimit` | `rate_limit` | The gate, when one joint stepped more than `max_step_rad`. |
| `NonFinite` | `non_finite` | A NaN or an infinity, or a broken constant. |
| `NoReference` | `no_reference` | The gate, when there is no last sent pose to compare with. |

`IkResult` is one leg. `ok` is true only together with `None`.
The comment on the struct is the contract: do not publish a result
whose `ok` is false.

Open `types.hpp` and find `leg_index`, `leg_id_ok`, and
`ik_reason_name`. There is no formula in the file. That is the point
of the sitting.

### Check

`test/test_geometry.cpp`, `RejectsALegIdOutsideOneToSix`. A leg id
outside 1..6 is refused at the table, rather than silently becoming
the front-left hip.

### Recap

- `Vec3` is millimetres in the body frame. `JointAngles` is radians before the pulse conversion.
- Leg 1 is index 0. `leg_index` is the subtraction.
- A refusal is an `IkReason` on a returned struct.

## Sitting 2 — The robot's numbers (`geometry.cpp`)

### The idea

The triangle formula is the same for any two-link leg. Masha enters
as six numbers that are not in the formula: where each hip is, how
long the links are, and what "zero" means on the servo. Those numbers
live in one file so a later file cannot hide a private copy.

### Where it sits

`hip(id)`, `link_lengths()`, `joint_zero()`, and `stand_tape_lf()` are
the only readers the other files are allowed. `hip()` aborts if the
id is not a real leg. Abort is a C++ stop of the process. A guessed
yaw would aim a foot along the wrong ray, which is worse than a crash
during a bug. Callers that hold an untrusted integer use
`hip_or_null` and do not publish on null.

### The mechanism

Body frame, millimetres. The hip xy values are `build_in_pose.py`'s
`X1`, `Y1`, `Y2`. The URDF `leg_center` of the front-left leg is a
different point, about `(104.6, 50.6, 16.6)` mm, because that point
belongs to the mesh. These hips belong to the simplified IK frame.
Lynch and Park, Chapter 4 opening, is why a CAD frame and an IK frame
may differ. This file does not solve the CAD chain.

Mount yaw is the heading of the foot when the coxa angle is 0.

| Leg | Hip x | Hip y | Mount yaw |
|---|---:|---:|---|
| LF | 93.60 | 50.805 | +45° (`π/4`) |
| LM | 0 | 73.535 | +90° |
| LR | −93.60 | 50.805 | +135° |
| RR | −93.60 | −50.805 | −135° |
| RM | 0 | −73.535 | −90° |
| RF | 93.60 | −50.805 | −45° |

`π/4` is written as `kPi / 4`, not as a rounded decimal. 45 degrees
is a quarter turn. The right side mirrors the left by negating y and
the mount yaw. There is one set of link lengths.

```
coxa_mm          45.0     yaw axis to femur hinge, horizontal
femur_mm         77.1     L1, femur axis to tibia axis
tibia_mm        115.6     L2, tibia axis to foot tip
hip_z_mm          0.0     the IK frame already shares z with the feet
coxa_femur_z_mm   1.13    femur hinge above the yaw-axis origin
```

The lengths started as distances between joint origins in
`rospider_description/urdf/base.urdf.xacro`. The round-trip test was
allowed to change them. It did not need to. The comment in
`geometry.cpp` says so.

Joint zero, shared by all six legs, fitted by
`LegIk.StandContract` in `test_leg_ik.cpp`:

```
coxa   0
femur  0.144427 rad    about 8.28°
tibia  1.481540 rad    about 84.89°
```

The tibia gap is large because the drawing's zero (straight) and the
servo's zero (centre of travel) are far apart. It is a definition of
zero. It is not a second copy of a link length, and the cosines do
not contain it.

`StandTape` is the vendor's left-front stand, about
`0.1207, 0.7555, -0.650` rad. It is the measuring tape the contract
compares against. Nothing adds it into a formula.

`hip()` returns `const Hip&`. The `&` is a C++ reference: a second
name for the row in `kHips`, not a copy, and not a pointer the caller
frees. The table lives for the whole process.

### Check

`test/test_geometry.cpp`: `HipTableMatchesBuildInPose`,
`LengthGuessesStaySeparate`, `JointZeroIsFittedAndStandTapeIsSeparate`.

On paper, confirm the front-left stand foot from the constants in
`test_leg_ik.cpp`:

```
x = 70 + 93.60 = 163.60
y = 110 + 50.805 - 20 = 140.805
z = -70
```

The coxa test rounds y to `140.8`. Both are the same foot at the
precision that test uses.

### Recap

- Hips and mount yaws are the body. Lengths are the links. The joint zero is the servo's ruler.
- One shared zero passed the 0.005 rad stand contract. A per-leg table was not required.
- The stand tape is a measurement. The formula does not add it.

## Sitting 3 — The coxa (`coxa.cpp`)

### The idea

The first hinge only turns left and right. Once it faces the foot,
the other two hinges live in a flat vertical plane. So the 3-hinge
question splits. This file is only "which way do I face?"

### Where it sits

`solve_leg` calls `solve_coxa` once per leg, then calls the triangle.
The gait loop does not call `solve_coxa` itself. Height is not an
input that changes the answer: a pure yaw ignores z. Whether the
links can reach the foot is the triangle's question, asked later.

### What you need

`atan2(y, x)` turns two sides of a right triangle back into an angle
and keeps the quadrant. `atan2(dy, dx)` is the heading from the hip
to the foot, measured from the body's +X. The servo's zero is not
the body's +X. Mount yaw is the direction the servo already points
at angle 0. Subtracting it is a change of reference direction in the
plane. Lynch and Park §3.1 is that idea. The `atan2` line in their
Chapter 6 opening (printed page 219) is the same function.

### The mechanism

```
dx = x_foot - hip_x
dy = y_foot - hip_y
q_coxa = wrap( atan2(dy, dx) - mount_yaw )
```

`std::atan2` takes y first and x second. That argument order is the
detail people swap. The comment in `coxa.cpp` writes the worked line
so you can check it on a calculator.

Left-front stand. Hip `(93.60, 50.805)`, foot `(163.6, 140.8)`:

```
dx = 70
dy = 89.995
atan2(89.995, 70) ≈ 52.12°
mount = 45°
leftover ≈ 7.12° = 0.1243 rad
```

The vendor tape for that foot is `0.1207` rad (6.91°). The test
window is 0.01 rad. No joint zero is added here. The coxa zero in
`geometry.cpp` is 0, and `solve_leg` is the place that would add it.

`wrap_pm_pi` puts the result in `(-π, π]`. `-π` is folded up to `+π`,
so each direction has one representative. `atan2` already returns a
value in that interval, and every mount yaw here is within ±135°, so
the difference lies in `(-2π, 2π)`. One addition or subtraction of a
full turn is the whole repair. A `while` loop would be the wrong
shape: a non-finite angle compares outside the interval forever.
A non-finite result is returned as it is. The gate, later, refuses it.

A foot sitting on the hip has no direction. `atan2(0, 0)` is 0, and
the mount yaw is then subtracted. That number is not a solved
heading. The triangle is what rejects a foot the links cannot reach.

### Check

`test/test_coxa.cpp`, in this order:

1. `FootOnTheLfMountRayIsZero` — a foot placed along +45° from the front-left hip has coxa 0.
2. `LfStandFootIsNearTheVendorTape` — the worked line above, within 0.01 rad of 0.1207.
3. `HeightDoesNotChangeTheYaw` — z = −70 and z = −40 give the same angle.
4. `LmHomeFootIsStraightOut` — the middle-left foot is on the +90° ray, so the angle is 0.
5. `LegSixMirrorsLegOne` — the front-right coxa is the negation of the front-left coxa.
6. `WrapsAHeadingPastPi` — a heading of `5π/4` comes back as `-3π/4`.

Do the first one on paper before you read the assertion. If your
calculator and the test disagree, the usual cause is degrees versus
radians, or `atan2` argument order.

### Recap

- Coxa angle = heading of the foot from the hip, minus the mount yaw, wrapped into `(-π, π]`.
- z is not part of this angle.
- This function always returns a number. "Can the leg reach?" is the next file.

## Sitting 4 — The knee (`planar_leg.cpp`)

### The idea

After the coxa faces the foot, femur and tibia are a triangle in a
vertical plane. You know the two link lengths and the place the tip
must land. The law of cosines turns those three lengths into the
angle at the knee. A triangle can be folded two ways. Both are
legal. A standing Masha keeps one of them, and that choice is a
separate function so the formula stays the formula.

### Where it sits

`solve_leg` builds a `PlaneTarget` and calls `solve_planar`. The
book's picture is Lynch and Park, Figure 6.1, printed pages 219–220.
Femur is `L1`, tibia is `L2`. The book's `(x, y)` is our
`(x_plane, z_plane)`. Jazar draws the same two folds as elbow-down
and elbow-up. Stop before Lynch §6.1; that section is a 6-joint
PUMA, a different machine.

### What you need

The law of cosines: for a triangle with sides `a, b, c`, the angle
opposite `c` satisfies `c² = a² + b² − 2ab cos(angle)`. Rearranged,
`cos` of that angle is known, and `acos` returns the angle. `acos`
is only defined for a cosine in `[-1, 1]`. A cosine outside that
interval means the three lengths are not a triangle. That is the
geometric meaning of "unreachable."

The reachable set, measured from the femur hinge, is a ring: no
closer than `|L1 − L2|`, no farther than `L1 + L2`. Lynch §2.5 and
Jazar §1.3.2 name that set a workspace.

### The mechanism, in the order the function runs

**1. Move the foot into the book's plane.** `plane_target`:

```
x_plane = hypot(x_foot - hip_x, y_foot - hip_y) - coxa_mm
z_plane = z_foot - hip_z_mm - coxa_femur_z_mm
```

`x_plane` is horizontal distance from the femur hinge out to the
foot. The coxa link has already been walked, so it is subtracted.
`z_plane` is the book's y. Up is positive. A foot below the femur
hinge has a negative `z_plane`. For the front-left stand this is
about `(69.0, −71.1)` mm. `coxa_femur_z` moves only this vertical
coordinate. It is not folded into `femur_mm` or `tibia_mm`.

**2. The two angles inside the triangle.**

```
r²    = x_plane² + z_plane²
cos β = (L1² + L2² − r²) / (2 L1 L2)
cos α = (r² + L1² − L2²) / (2 L1 r)
γ     = atan2(z_plane, x_plane)
```

`β` is the interior angle at the knee, opposite `r`. At full stretch
`β` is `π` and the two folds meet. `α` is the angle between the femur
and the line to the foot. `γ` is the heading of that line, up from
horizontal. A foot below the hinge gives a negative `γ`.

**3. The two folds.**

```
righty:  θ1 = γ − α,   θ2 =  π − β
lefty:   θ1 = γ + α,   θ2 =  β − π
```

`θ1` is the femur, measured from horizontal out along `+x_plane`.
`θ2` is the tibia, measured from the femur. Zero tibia is straight.
A negative tibia bends the other way.

**4. Which fold stands.** `standing_branch()` returns `Lefty` and
does not look at the foot. The comment above it records the decision
from the front-left stand foot `(163.6, 140.8, −70)`:

```
lefty:   femur ≈ +0.61 rad (+35°),  tibia ≈ −2.13 rad (−122°)
righty:  femur ≈ −2.21 rad,         tibia ≈ +2.13 rad
```

A standing Masha holds the femur up and the tibia down, so the kept
fold is lefty. Raising the foot from z = −70 to z = −50 moves lefty
to about `+0.94` and `−2.30`: the femur rises and the tibia bends
further down. `RaisingTheFootLiftsTheFemur` locks that direction.

**5. Add the joint zero, after the book angles exist.**

```
femur_servo = θ1 + 0.144427
tibia_servo = θ2 + 1.481540
```

The vendor tape for the stand is about `+0.76` and `−0.65` rad. The
gap between the book angles and that tape is this zero. The cosines
do not contain it. `JointZeroIsAddedAfterTheBookAngles` locks the
order: change the zero, and the book angles stay, the servo angles move.

**6. Refuse a cosine outside `[-1, 1]`.** Reason `unreachable`. The
angles stay 0. That 0 means "not computed." Nothing is clamped onto
the ring.

**7. Refuse the soft edge.** Just inside the ring the knee rate blows
up. The derivative used here is

```
|dβ/dr| = r / (L1 L2 |sin β|)
```

At full stretch `β` is `π`, `sin β` is 0, and any positive foot error
asks for an unbounded knee step. `δ = 0.5` mm is the foot error the
forward round trip is required to survive. If `δ` times that
derivative is greater than `max_step_rad`, the reason is
`near_singular`. The cutoff is that product, not a fixed cosine such
as 0.99.

The comment in `safety.cpp` puts numbers on it. At the front-left
stand, 0.5 mm asks the knee for about 0.0066 rad, inside the limit.
The same error asks for about 0.116 rad when the foot is 0.2 mm short
of straight, still under the limit, and about 0.37 rad when the foot
is 0.02 mm short, over the limit. `JustInsideTheOuterRingIsNearSingular`
uses that closer foot.

On `near_singular` the angles are filled, because the triangle
exists, and `ok` is false. Callers may log them. They must not
publish them. `α` grows at the same edge. The rate gate is the
backstop for `α` and for the coxa. This file's soft edge is about `β`.

A length that is not a positive number of millimetres, or a step
limit that is not a positive angle, is `non_finite`, before `acos` runs.

### Check

`test/test_planar_leg.cpp`. Read `HandBuiltEquilateralMatchesTheBook`
first: an equilateral triangle has a known answer, so a failure there
is the formula, not Masha's link lengths. Then
`OutsideTheOuterRingIsUnreachable`, `InsideTheInnerHoleIsUnreachable`,
`StandFootKeepsLeftyAndClearsTheSoftEdge`, and
`CoxaFemurZMovesOnlyTheBookVertical`.

### Recap

- `plane_target` moves the foot from the body to the book's `(x, y)`.
- Law of cosines gives `α` and `β`. The two folds are `γ ± α`.
- Standing keeps lefty, then adds the joint zero. Outside the ring is unreachable. Just inside the straight edge is near-singular.

## Sitting 5 — Forward, the check (`forward.cpp`)

### The idea

Inverse kinematics answers "what angles put the foot here?" Forward
kinematics answers "if the hinges sit at these angles, where is the
foot?" If you run inverse and then forward, the foot you get back
should be the foot you started with, inside the region where a
triangle exists. That round trip is the statement that the two maps
undo each other. The gait loop does not call this file. The tests do.

### Where it sits

Kala §1.4.3, Figure 1.8, is the picture: angles in, tip out. Lynch
Chapter 4 opening names the map. Jazar §5.3 is the classical chain,
a Denavit–Hartenberg table. This file does not build that table. The
leg is a yaw, then two links in the vertical plane that yaw faces.
Three geometric steps are the whole function. Product of exponentials
is the rest of Lynch Chapter 4, and it is not this code.

### The mechanism

The angles that arrive still carry the joint zeros. Those zeros are
subtracted first, because the drawing measures the femur from
horizontal and the tibia from straight.

```
q_coxa = coxa_rad - zero.coxa
θ1     = femur_rad - zero.femur
θ2     = tibia_rad - zero.tibia

heading = mount_yaw + q_coxa
hinge   = hip + coxa_mm along that heading, and coxa_femur_z along +Z

x_plane = L1 cos θ1 + L2 cos(θ1 + θ2)
z_plane = L1 sin θ1 + L2 sin(θ1 + θ2)

foot    = hinge + x_plane along the heading, and z_plane along +Z
```

Zero and zero in the drawing is straight out: femur horizontal,
tibia continuing in the same direction. A wrap of `heading` into
`(-π, π]` would not move the foot, because `cos` and `sin` already
have period `2π`. The wrap belongs to the coxa solver, where the
number is an angle a person compares. Here the number is an input
to `cos` and `sin`.

The straight-leg test sets the servo angles equal to the joint zero,
so the drawing angles are `0, 0, 0`. Reach along the mount ray is
`45 + 77.1 + 115.6 = 237.7` mm. The foot's z is `1.13`, which is
`coxa_femur_z`. A straight leg does not cancel that gap.
`StraightLegLandsOnTheMountRay` locks the three coordinates.

`RoundTripAroundDefaultPose` takes a grid around the stand, six
legs, ±40 mm in x and y, z from −100 to −40. A point the solver
marks unreachable or near-singular is skipped. Every point it
accepts must come back within 0.5 mm. That 0.5 mm is the same `δ`
the soft edge uses. The two files agree on what "close enough" means.

### Check

Read `forward_foot` once with the straight-leg comment beside it.
Then read the round-trip loop and notice which points it skips, and
why a skipped point is not a failed test.

### Recap

- Forward subtracts the joint zero, walks the coxa, then walks femur and tibia in that plane.
- Inverse then forward must land within 0.5 mm on the feet the solver accepts.
- The gait loop does not call this. The test is the caller.

## Sitting 6 — One leg, then six (`leg_ik.cpp`)

### The idea

`solve_leg` is the composition. There is no third formula. It calls
the yaw, then the triangle, and adds the coxa zero. `solve_pose` is
the call one 20 ms tick makes: six feet in, one decision out.

### Where it sits

The Python binding calls `solve_pose`. `solve_pose` calls `solve_leg`.
`gtest` calls both directly. A refusal stays a filled `PoseResult`.
The function does not throw, because a throw in the binding would
drop the gait generator.

### The mechanism

`solve_leg`, in order:

1. A non-finite foot coordinate returns `non_finite` with angles left at 0.
2. `solve_coxa`. A non-finite yaw returns `non_finite`. `hip()` aborts if the id is not a real leg, before this point.
3. `plane_target`, then `solve_planar`. If the knee's `ok` is false, the leg result copies the knee's reason and returns. Angles stay 0.
4. On success, coxa is `q_coxa + zero.coxa` (the zero is 0 today; the add stays so a later fit has one place to land). Femur and tibia are the standing angles, which already include their zeros.

`solve_pose` walks LF, LM, LR, RR, RM, RF. The first refusal is the
pose's refusal, and the loop stops. Legs already solved keep their
angles, so a log can see how far the tick got. The refusing leg and
every later leg stay 0. `ok` false keeps all of them off the servos.
A later bad foot does not replace the leg id. When `ok` is true,
`reason` is `None` and `leg` is unused.

`PoseResult::angles` is `std::array<JointAngles, 6>`. `angles[0]` is
leg 1. That is the kinematic order `JointControl` also uses: joint 1
is the front-left coxa, joint 18 is the front-right tibia.

`max_step_rad` is passed through to every leg, because the soft edge
needs it. The measured value belongs to `safety.cpp`. Tests pass the
same function the gate uses, so the test and the robot share the
cutoff.

### Check

`test/test_leg_ik.cpp`, `StandContractWithinHalfAHundredthOfARadian`.
This is the contract the joint zero was fitted to. All six
`DEFAULT_POSE` feet, within 0.005 rad of the radians
`set_leg_position` returns. The targets are written in `kStand` with
the degrees beside them. Corner coxa `0.1206814507` rad is the plan's
`0.1207`. Middle legs are a different triangle, about 4 mm closer in
the plane, so their vendor femur and tibia differ. The right side
negates the coxa and keeps the femur and tibia of its left mirror.

Then `AFarFootIsUnreachableAndDoesNotThrow`: the same front-left xy
with z = 500 returns `unreachable`, femur and tibia exactly 0, and
does not throw.

`test/test_solve_pose.cpp`:

- `DefaultPoseMatchesEachSolveLeg` — the six-leg call agrees with six one-leg calls.
- `FirstBadLegFailsTheWholePose` — the reported leg is the first refusal, and later legs stay 0.
- `AStraightFootIsNearSingularAndNamesThatLeg`.
- `ANonFiniteFootNamesThatLegAndDoesNotThrow`.

### Recap

- `solve_leg` is coxa, then the triangle, then the coxa zero.
- `solve_pose` stops at the first bad leg and still returns a struct.
- The stand contract is 0.005 rad on all six feet against the vendor tape.

## Sitting 7 — The gate (`safety.cpp`)

### The idea

The solver has named eighteen angles. A second question remains: given
those angles and the angles last actually sent, may this pose be
published? The gate answers once for the whole pose. It does not fold
a triangle, pick a foot, clamp a hinge, or invent a shorter step. A
shorter step is the caller's later repair, in `approach_frame`.

### Where it sits

`HexapodLegIk.solve` calls `gate` after `solve_pose`. The previous
pose is owned by the Python object. The controller's `joints_state`
starts at zero and is not this argument. Passing zeros would make the
first real step look like a jump away from the origin.

The ±120° window is the driver's travel, copied from the leg rows of
`kinematics/config.py`. The gate uses the same comparison
`JointControl` uses:

```
| direction * (offset + radians) |  >  | max_radians / 2 |
```

Leg servos are 240° of travel around pulse 500, so the half-window
is ±120°, about ±2.094 rad. Offsets in that table are 0. Directions
are copied per joint, including the right-side femur and tibia signs,
because a later non-zero offset would shift the window on the side
the driver actually uses. Head joints 19 and 20 are not legs and are
not in this gate.

### The mechanism

Checked in this order. The first failure is the decision. A later
check does not get a vote.

1. Any non-finite angle, in the command or in the previous pose, is `non_finite`, naming that joint. A NaN hides a later window problem on purpose: the number is not an angle, so the window is meaningless.
2. `command.ok` false keeps the solver's reason. `joint_id` stays 0. The failure is a leg, and `command.leg` already names it. The gate does not re-label an unreachable foot as a window or a step.
3. A joint outside the driver window is `joint_limit`, naming it. The angle is not pulled back to ±120°.
4. `previous == nullptr` is `no_reference`. The step check would otherwise subtract zero and treat an unsolved reference as a real stance.
5. A joint step larger than `max_step_rad` is `rate_limit`, naming the joint and both angles. Equal to the limit is allowed. The comparison is `>`.

Otherwise `allow` is true and `reason` is `None`.

`max_step_rad()` returns `kMaxStepRad`:

```
0.17622116866774196 * 1.5 = 0.26433175300161293
```

The peak is a measurement, written up in `scripts/calibrate_max_step.py`.
That script asks the vendor solver, with no servos, for one
`FollowGaitGenerator` cycle from `DEFAULT_POSE`: hunter lift 35 mm,
period 0.60 s, Twist clamps `vx = +0.12` m/s, `vy = +0.10` m/s,
`wz = +0.6` rad/s, thirty frames. The largest `|Δq|` in one tick of
that cycle is `0.17622116866774196` rad, on `femur_LF` (joint 2), the
step into frame 5. The other seven sign combinations of those clamps
land on the same peak, mirrored onto another femur. The constant is
that peak times 1.5.

The first tick of that same command, from the stand into the intro
splice, is about 0.328 rad. That is a catch-up, which is why the
limit is not raised to cover it. A knee flip is still about a radian,
and 1.5 times the measured peak still refuses a flip.

The startup line prints this number, and prints the peak as the
number divided by 1.5. On the hunt, that line was
`max_step_rad=0.26433175300161293 measured_peak=0.17622117`.

`SafetyDecision` carries `allow`, `reason`, `joint_id`, and the pair
`q_now_rad`, `q_previous_rad` that a rate-limit line prints.
`joint_id` is 1..18 when one joint caused the refusal, and 0 when
the refusal is the whole pose or when allow is true.

The solver's word is `ok`. The gate's word is `allow`. A pose with
`allow` false publishes nothing. Both can be false. `ok` true and
`allow` false is a legal foot whose step or window was refused. That
is the case the catch-up knows how to repair, and only when the
reason is `rate_limit`.

### Check

`test/test_safety.cpp`. The order of the tests is the order of the
checks:

- `ServoTableMatchesTheDriverLegRows`
- `ANanIsNonFiniteBeforeTheWindow`
- `ANonFiniteAngleWinsOverASolverRefusal`
- `SolverNearSingularIsKept`
- `AJointPastTheHalfWindowIsJointLimit` and `ExactlyTheHalfWindowIsInside`
- `AWindowFailureWinsOverAMissingReference`
- `NoPreviousSentPoseIsNoReference`
- `AOneRadianJumpIsRateLimit`, `AStepOfAFewHundredthsIsAllowed`, `AStepEqualToTheLimitIsAllowed`

Read `calibrate_max_step.py` far enough to see that it calls the
vendor solver and prints a peak. It does not edit `safety.cpp` by
itself. A person copies the peak into `kMaxStepRad`.

### Recap

- The gate is five checks, first failure wins: finite, solver ok, ±120°, a previous pose, then the step limit.
- The step limit is 1.5 times a measured femur peak, `0.26433175300161293` rad.
- The gate does not shorten a step. `rate_limit` is a no, plus the numbers the catch-up needs.

## Sitting 8 — The Python door (`bindings.cpp`) and the build

### The idea

The 20 ms loop speaks Python lists of numbers. `solve_pose` and
`gate` speak C++ structs. `bindings.cpp` carries the numbers across
and carries the decision back. No triangle lives here.

### Where it sits

`import hexapod_kinematics` loads the pybind11 module. One tick calls
`solve_pose`, then `gate`. pybind11 is a C++ library that builds a
Python extension. It is not a ROS package. The module is conversion
only.

### What you need

Two files get built, on purpose, so their names do not collide:

| File | What it is |
|---|---|
| `libhexapod_kinematics.so` | The C++ library: geometry, coxa, planar, forward, solve, gate. |
| `hexapod_kinematics.cpython-310-aarch64-linux-gnu.so` | The Python module. It links the C++ library. |

`WITH_SOABI` is why the Python file carries `cpython-310-aarch64`.
That suffix is the interpreter and the CPU, so a module built for
this Jetson is not loaded by a different Python.

`INSTALL_RPATH "$ORIGIN/…"`. `$ORIGIN` is the directory of the Python
module itself. The relative step, computed in `CMakeLists.txt`,
reaches `lib/` where the C++ library is installed. After you
`source` the workspace, `import hexapod_kinematics` finds both files
from that path. The comment in the CMake file says what fails without
the environment hook: the import cannot see the module.

Compile flags: `-O3` so `hypot`, `atan2`, and `acos` can inline.
`-ffast-math` is deliberately absent. Fast-math is allowed to break
NaN checks and to treat a cosine outside `[-1, 1]` loosely. The gate
and the knee both depend on those checks being real.

C++17. No `rclcpp`. `ament_cmake` is here so the package installs and
tests the way the rest of the workspace does. The math does not call
it.

### The mechanism

`solve_pose(feet)` accepts six sequences of three numbers. A string,
a wrong count, or a foot that is not three numbers raises
`ValueError`. The message names the slot (`foot x is a number of
millimetres`). An int is accepted, because a millimetre written as
`70` is still a number. The C++ call uses `max_step_rad()` inside the
binding. The tick does not pass a second copy of the limit.

The returned `PoseResult` exposes `ok`, `reason` as the English
string, `leg` as an int 1..6, and `angles` as six triples.

`gate(command, previous)` accepts the previous pose as eighteen
radians, or as six triples. `None` means there is no previous pose,
and the binding passes a null pointer. `command_from_angles` builds
an `ok` command without solving a foot, so a test can offer the gate
a pose that did not come from `solve_pose`. The tick uses `solve_pose`.

A foot the solver refuses comes back with `ok` false. A pose the
gate refuses comes back with `allow` false. Neither call raises for
those answers.

### Check

`test/test_bindings.py`:

- `test_stand_matches_the_vendor_tape`
- `test_a_far_foot_returns_unreachable_and_does_not_raise`
- `test_the_wrong_number_of_feet_raises`
- `test_gate_allows_a_repeated_stand_from_triples_or_eighteen`
- `test_no_previous_pose_is_no_reference`
- `test_a_one_radian_jump_is_rate_limit`
- `test_a_joint_past_the_window_is_joint_limit`
- `test_near_singular_reason_is_kept`

From a sourced shell, the package tests are:

```
cd ~/ros2_ws
colcon test --packages-select hexapod_kinematics --event-handlers console_direct+
```

The C++ tests do not need a running robot. The binding tests do not
publish a pulse.

### Recap

- `bindings.cpp` converts lists and returns decisions. A bad foot is a result. A bad shape is `ValueError`.
- Two `.so` files: the C++ library, and the Python module that links it.
- No fast-math, because NaN and a cosine outside `[-1, 1]` are real answers.

## Sitting 9 — The plug (`controller/leg_ik.py`)

### The idea

Two formulas sit behind one call. `make_leg_ik` picks one when a
gait-5 generator starts, and that object stays until the generator
is replaced. A parameter change in the middle of a step must not
swap the formula under a moving foot.

### Where it sits

This file is in the `controller` package, not in `hexapod_kinematics`.
It does not publish a pulse. `StepController` reads the ROS parameter
and calls `start_gait5`. Joystick gaits, voice gait 2, body lean, and
action groups do not call this file. They stay on `set_pose_base`.

`USE_HEXAPOD_KINEMATICS_DEFAULT` is `False`. That Python constant is
the default of the ROS parameter. The value that matters at runtime
is the parameter.

### The mechanism

**VendorLegIk.** `kinematics.set_leg_position`, one leg at a time.
The import of `kinematics.so` happens on the first solve, so
constructing the object does not load the binary. This backend does
not run the new rate gate. `JointControl` still enforces the ±120°
window on the way to the servo. Its `IkDecision` is always allow.

**HexapodLegIk.** `hk.solve_pose(pose)` then `hk.gate(result, previous)`.
The import happens on the first solve. `establish_reference` is the
special case at the start of a bout: the gate answers `no_reference`
when previous is empty, even for a legal stand. The stand is compared
with itself, so the window is checked and the step is zero. Those
angles become `previous`. Nothing is published: the feet are already
there. A stance the backend refuses leaves `previous` empty, and the
next frame holds with that reason.

**approach_frame** is the only place that turns one `rate_limit` into
a shorter step. The order is fixed.

1. Solve the generator's own feet. If the gate allows them, this tick is `sent`, `alpha` becomes 1, and the next tick may ask the generator for a new frame.
2. If the only failure is `rate_limit`, the feet are legal and the step is too big. Eight bisections search the open interval past `alpha_done`. One fraction is shared by all six legs:

   ```
   feet(alpha) = anchor + alpha * (frame - anchor)
   ```

   A per-leg fraction would twist the body. The largest alpha that passes the whole gate is a `partial`. The generator stays on this frame.
3. Any other reason is a hard hold. Nothing is published. `alpha_done` and the previous sent pose stay where they are.
4. A `rate_limit` whose search finds nothing is the same hold. A branch flip is this case: the foot barely moves and every fraction still jumps by about a radian, because the standing branch does not change along the segment. Catch-up must not publish that jump.

`alpha` becomes 1 only when step 1 passes. Eight bisections land on
a 1/256 grid of the remaining segment, so `0.996` is still a partial.
Rounding that up to 1 would start the next frame while these feet
were still short, and it would do it without the gate having allowed
this frame. A candidate less than one part in 256 past `alpha_done`
is no progress, and that tick is a hold.

The partial row still says `rate_limit`, and it names the joint of
the generator's own feet. The angles stored on a partial row are the
angles that were sent. The WARN names the generator's foot, because
that is the foot that was refused.

This pause is for a tripod. Three feet stay on the ground while one
frame is stretched over several ticks, so the body is not depending
on a swing finishing on time. A gait that stays upright by momentum
would fall if a swing were slowed this way. That gait is outside
this module.

**Gait5Session** is one generator and the film frame it has not
finished. `send()` on the generator yields six feet and moves the
cursor. While `waiting()` is true the loop keeps those feet and does
not call `send()`. `open_frame` stores `last_part` and does not act
on it. The boundary runs only when `step()` reports the frame
accepted at alpha 1. Honoring `last_part` early would start the next
cycle while the feet were still short.

**odometry_increment** scales the body twist by `delta_alpha`. A hold
adds nothing. The tick that reaches alpha 1 adds only the leftover
fraction. It does not repay the seconds the partials already covered.
The yaw rotation into the world frame stays in the 20 ms loop. This
function does not know the current yaw.

**TraceWriter** is a second thread, the only one that writes the CSV.
The 20 ms loop calls `put` with `block=False`. A full queue (200
rows) drops the row and warns once a second. A blocking put would
stop the tick, which is the stall this thread exists to avoid. The
file is appended, with a header when it is created. Flush is every
25 rows or every 0.5 s. `close` waits at most 0.2 s, so a halt is
not held up by the disk. The path is
`/home/ubuntu/ros2_ws/log/hexapod_ik_trace.csv`.

**HoldWatch.** A partial is not a hold. Five holds in a row is 100 ms
and logs one ERROR: `holding last safe pose, generator frame frozen`.
The generator is not dropped. A halt is what clears a hard hold. The
hunt you ran produced no ERROR and no `held` row.

**SecondLog.** One INFO per second: backend, last decision, largest
`|dq|`. The 50 Hz detail stays in the CSV.

### Check

`controller/test/test_leg_ik.py` and `controller/test/test_gait5_session.py`.
Read a test whose name is the behaviour you just read: a rate limit
becomes a partial, a one-radian flip stays a hold, a parameter change
does not rebuild the session already walking.

Then open the trace from the hunt and pick one `partial` row and the
`sent` row that follows it. Confirm the partial alpha is below 1, the
reason is `rate_limit`, the joint is a femur, and the following sent
row has an empty reason and eighteen `dq` values each at most
`0.26433175300161293` in absolute value.

### Recap

- One generator, one backend object. The flag is read when the generator starts.
- `rate_limit` is walked in smaller pieces along the straight line from the last sent feet. Every other refusal holds.
- The trace thread can drop a row. The 20 ms thread does not wait on the disk.

## Sitting 10 — The 20 ms loop (`step_controller.py`)

### The idea

Something has to call the session once per tick, publish only when
the session says so, and keep the generator from advancing while a
frame is unfinished. That something is the loop that already walked
every other gait.

### Where it sits

`StepController.loop` is a daemon thread that sleeps 0.02 s. It is
not a ROS timer, and it is not an rclcpp callback. ROS 2 delivered
the Twist earlier, on a subscription, and that callback only queued a
generator. This thread does the IK and the pulse publish. You are
allowed to treat "callback group" and "executor" as later vocabulary.
What matters here is: one thread, one lock around the queued
commands, and 20 ms between iterations even during a 50 ms blend.

The parameter is declared in `__init__` unless a launch override
already declared it. Declaring it twice raises.
`automatically_declare_parameters_from_overrides` is why a launch
argument of the same name is already present.
`ros2 param set /step_controller use_hexapod_kinematics true` changes
it for the next generator, not for the session already walking.

### The mechanism

On each iteration, for a current moving generator:

- If a gait-5 session is `waiting()`, reuse the held feet. Do not call `send()`.
- Otherwise `send()` the generator. That yields six feet, `last_part`, the params, and the slow word.
- If `params` is a `CmdVelParams` and `params.gait == 5`, this is the hunter follow walk.
  - `_begin_gait5` creates the session on the first tick and logs `gait 5 backend=...`.
  - `open_frame` on a freshly yielded pose.
  - `session.step()`, then `_record_gait5` (the WARN, the one-hertz line, the CSV row).
  - `honor_boundary` only when `step()` says the frame finished and this yield was `last_part`.
  - Duration 0.05 s on the first slice of a `cmd_true` blend, otherwise 0.02 s. The thread still sleeps 0.02 s.
  - `should_publish`: a hold does not publish. A dry run (`pseudo`) does not publish either.
  - `_publish_leg_radians` hands eighteen radians, ids 1..18, to `JointControl`. It does not call `kinematics.so`, and it does not replace `self.pose`.
  - `_integrate_fraction` adds `twist * 0.02 * delta_alpha` to the odometry.
- Any other gait calls `set_pose_base`, which calls `kinematics.set_leg_position` six times.

`_drop_gait5_if_replaced` forgets the session when `cur_moving_generator`
is no longer the session's generator. The next gait-5 start reads the
flag again. A pose-set, a halt, and a finished boundary are the
replacements.

`set_pose_base` and `set_leg_position` are still the vendor path.
Their comments in the file still talk about pulses in places. The
return value of `set_leg_position` is radians. `JointControl` is what
multiplies by direction and writes the pulse. The gait-5 branch does
not go through those two methods.

An exception in the moving-pose handler logs `MOVING_POSE` and drops
the generator. That is why the library returns a refusal instead of
throwing. A programming bug (the wrong shape of feet) still throws,
and dropping the generator is the right outcome for a caller bug.

### Check

Read the block that starts at the comment "Gait 5 is the hunter
follow walk." Say, for one iteration, which function moves the
generator cursor, which function decides publish, and which function
writes a pulse.

Then read `honor_boundary` and the comment above it. A gait-5 partial
does not call it.

`controller/test/test_gait5_session.py` asserts the parameter name
is in the controller source. That test is a lock on the wiring, not
a walk on the robot.

### Recap

- The loop is a 20 ms Python thread. Gait 5 goes through `Gait5Session`. Every other gait goes through `set_pose_base`.
- An unfinished frame is reused. `send()` waits until alpha is 1.
- Publish is eighteen radians into `JointControl`, and only when the session's `publish` flag is true.

## Sitting 11 — Read the hunt as a worked example

You already have the numbers. This sitting is to connect them to the
functions, with the CSV open.

1. Find the startup line. It was false, and it printed the step limit. That print is `read_max_step_rad` importing the module at process start. It is not the backend of the walk.
2. Find the first `gait 5 backend=hexapod`. That is `_begin_gait5` → `start_gait5` → `begin_generator`. From there to the end of that log, count vendor. The count is zero.
3. Pick a second-log line `backend=hexapod decision=sent max_dq=0.2642`. That `max_dq` is the largest absolute joint step `SecondLog` saw in that second. It sits just under `max_step_rad()`. A vendor walk would not be clipped to this number, and it would not print `backend=hexapod`.
4. In the CSV, filter `decision == partial`. Confirm every `reason` is `rate_limit` and every `joint` is a femur (2, 5, 8, 11, 14, or 17). That is `approach_frame` step 2, and it matches the calibration sentence that the peak lives on a femur.
5. Take one partial whose alpha is `0.99609375`. That is 255/256. The next rows of that bout should reach a `sent` with alpha 1. The generator was held on that frame until the gate allowed the generator's own feet.
6. Confirm there is no `held` row and no `holding last safe pose` ERROR. The hunt never asked for an unreachable foot, a near-singular knee, or a joint past ±120°. The only refusal was "this legal step is too big for one tick," and the catch-up finished it.

### Recap

- `backend=hexapod` plus `partial` / `rate_limit` is this library, including the gate.
- The startup flag is the value at process creation. The generator line is the value that chose the formula.
- After a restart the parameter comes back false unless it is set again.

## Sitting 12 — The books, beside the file you are in

Read the row in [PLAN.md](PLAN.md), section "Which file to open with
which pages," before you re-read that file. Short version, so this
path stays attached to the code:

| When you are in | Read |
|---|---|
| `geometry` | Kala §1.4.1–§1.4.2. Lynch §3.1. Jazar §1.4.3. The hip numbers are this robot. No chapter lists them. |
| `coxa` | Lynch §3.1, then `atan2` on printed page 219. |
| `planar_leg` | Lynch Figure 6.1, printed pages 219–220. Then Jazar Figures 6.1 and 6.3. Jazar §6.5 is the straight-leg edge. |
| `forward` | Kala §1.4.3. Lynch Chapter 4 opening. |
| `leg_ik` | No new chapter. It calls the two files in order. |
| `safety` | Lynch §2.5. Jazar §1.3.2, §6.4.1, §6.5. The ±120° window is the driver. The step limit is the measurement. |

Kala Chapter 1 §1.4 is the on-ramp and stops at the end of §1.4.3.
§1.4.4 is a rolling car. Denavit–Hartenberg is named so you recognise
it. This leg does not fill in a DH table. Nothing in the formulas
was taken from `kinematics.so`. There is no source to take it from.

## Say it back

When you can say these without the files open, you have the module.

1. A gait decides six foot tips. This library turns one tick's tips into eighteen radians and a yes or a no.
2. The body frame is +X head, +Y left, +Z up, millimetres.
3. The coxa is `atan2(dy, dx) - mount_yaw`, wrapped into `(-π, π]`.
4. The knee is Figure 6.1. Standing keeps the lefty fold, then adds the joint zero.
5. Outside the ring of `L1` and `L2` is unreachable. Just short of straight, if 0.5 mm would swing the knee past the step limit, is near-singular.
6. Forward walks the three links and must land within 0.5 mm of a foot the solver accepted.
7. `solve_pose` stops at the first bad leg and returns the reason. It does not throw.
8. The gate's order is finite, solver ok, ±120°, a previous pose, then one step of at most `0.26433175300161293` rad.
9. Only `rate_limit` is subdivided. The fraction is shared by all six feet. Alpha becomes 1 only when the generator's own feet pass.
10. Any other refusal holds the last safe angles and holds the generator on that frame.
11. `JointControl` is what turns an allowed radian into a pulse. This library does not.
12. The flag is read when a gait-5 generator starts. The hunt from 14:43 to 14:58 logged `backend=hexapod` on every one of those generators.
13. `kinematics.so` still answers every other gait, and it answers gait 5 when the flag is false.
14. The trace is a second thread. The 20 ms loop only queues a row.
15. The tests are the round trip, the stand contract, and the order of the gate. A failing test names the file that owns the subject.

## If you get lost in a function

Ask three questions, in this order.

1. Which frame is this number in: body millimetres, the book's plane, a drawing angle, or a servo radian?
2. Which file owns it: geometry, coxa, the triangle, forward, the composition, or the gate?
3. Did the gate allow it? If the reason is anything except an empty string, the pulse path does not run.

The answer to those three is the function. The rest is the arithmetic you already met in that file.
