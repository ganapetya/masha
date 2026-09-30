# hexapod_kinematics

Open-source inverse kinematics for one case: Masha's hunter gait 15.

The closed library `kinematics.so` still answers every other walk. This package will answer gait 15 when a flag is on. The flag defaults to off.

**This commit is the plan only.** The solver is not here yet. Read [PLAN.md](PLAN.md). Work order item 1 is this package. The math starts after this commit.

The reading that sits beside each future file is in PLAN.md under "Materials this plan follows." Three books: Kala's Chapter 1 §1.4 is the on-ramp, Lynch and Park's Figure 6.1 is the formula, and Jazar is the second pass (elbow up / elbow down is the same pair of folds as lefty / righty). The PDFs are not in this package.
