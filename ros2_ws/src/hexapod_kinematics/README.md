# hexapod_kinematics

Open-source inverse kinematics for one case: Masha's hunter gait 15.

The closed library `kinematics.so` still answers every other walk. This package will answer gait 15 when a flag is on. The flag defaults to off.

**The tree so far is the plan plus the geometry skeleton.** `include/hexapod_kinematics/types.hpp` names a point, three angles, and a yes-or-no result. `geometry.hpp` / `geometry.cpp` hold the hip table, the length guesses, and `coxa_femur_z`. There is no coxa solver yet. Read [PLAN.md](PLAN.md). Work order item 2 is this skeleton. The triangle starts at the next step.

The reading that sits beside each future file is in PLAN.md under "Materials this plan follows." Three books: Kala's Chapter 1 §1.4 is the on-ramp, Lynch and Park's Figure 6.1 is the formula, and Jazar is the second pass (elbow up / elbow down is the same pair of folds as lefty / righty). The PDFs are not in this package.
