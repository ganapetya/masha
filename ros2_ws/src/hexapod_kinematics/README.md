# hexapod_kinematics

Open-source inverse kinematics for one case: Masha's hunter gait 15.

The closed library `kinematics.so` still answers every other walk. This package will answer gait 15 when a flag is on. The flag defaults to off.

**The tree so far answers one tick and decides whether it may be sent.** `geometry` holds the hips, the link lengths, and the joint zero fitted to the stand. `coxa` is the yaw. `planar_leg` is the knee triangle. `forward` walks those angles back to a foot. `solve_leg` calls the yaw, then the triangle, for one foot. `solve_pose` calls that six times and returns one decision. `safety` says whether that decision may reach a servo. Read [PLAN.md](PLAN.md). Work order item 7 is this step. The Python binding is the next step.

The reading that sits beside each future file is in PLAN.md under "Materials this plan follows." Three books: Kala's Chapter 1 §1.4 is the on-ramp, Lynch and Park's Figure 6.1 is the formula, and Jazar is the second pass (elbow up / elbow down is the same pair of folds as lefty / righty). The PDFs are not in this package.
