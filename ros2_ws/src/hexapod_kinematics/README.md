# hexapod_kinematics

Open-source inverse kinematics for one case: Masha's hunter gait 15.

The closed library `kinematics.so` still answers every other walk. This package will answer gait 15 when a flag is on. The flag defaults to off.

**The tree so far is the plan, the geometry skeleton, and the coxa yaw.** `include/hexapod_kinematics/types.hpp` names a point, three angles, and a yes-or-no result. `geometry.hpp` / `geometry.cpp` hold the hip table, the length guesses, and `coxa_femur_z`. `coxa.hpp` / `coxa.cpp` turn a foot's horizontal position into one yaw angle. There is no knee triangle yet. Read [PLAN.md](PLAN.md). Work order item 3 is the coxa. The triangle is the next step.

The reading that sits beside each future file is in PLAN.md under "Materials this plan follows." Three books: Kala's Chapter 1 §1.4 is the on-ramp, Lynch and Park's Figure 6.1 is the formula, and Jazar is the second pass (elbow up / elbow down is the same pair of folds as lefty / righty). The PDFs are not in this package.
