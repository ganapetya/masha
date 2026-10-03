# hexapod_kinematics

Open-source inverse kinematics for one case: Masha's hunter gait 15.

The closed library `kinematics.so` still answers every other walk. This package answers gait 15 when `use_hexapod_kinematics` is true. The flag defaults to false. A gait-5 generator reads it when that generator starts.

`StepController.loop` calls the plug for gait 5. `geometry` holds the hips, the link lengths, and the joint zero fitted to the stand. `coxa` is the yaw. `planar_leg` is the knee triangle. `forward` walks those angles back to a foot. `solve_leg` calls the yaw, then the triangle, for one foot. `solve_pose` calls that six times and returns one decision. `safety` says whether that decision may reach a servo. `bindings.cpp` is the door `import hexapod_kinematics` opens. `controller/leg_ik.py` is the two backends and the catch-up search. Read [PLAN.md](PLAN.md) for the books and the design. Read [learning-from-code.md](learning-from-code.md) to study the code in order.

The reading that sits beside each file is in PLAN.md under "Materials this plan follows." Three books: Kala's Chapter 1 §1.4 is the on-ramp, Lynch and Park's Figure 6.1 is the formula, and Jazar is the second pass (elbow up / elbow down is the same pair of folds as lefty / righty). The PDFs are not in this package.
