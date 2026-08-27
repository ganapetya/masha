# Current plan (recognition only)

**As of:** 2026-08-27  
**Active:** Quarter 1, Week 1 — The Coordinate & Memory Foundation  
**Status:** In progress. Today is part of Week 1. We work on this plan. **Do not implement until Peter asks. Do not write the result file until the week is complete.**

## Two levels of plan

In `learning-bots-sharing` we keep **quarter plans** and **week plans** (weeks live inside a quarter). A quarter file is a hub, not a dump of one week’s lessons.

| File | Role |
|------|------|
| `quarter-01-plan.md` | Quarter 1 tracker only: title *Foundations of Spatial Mechanics & Edge CV*, weeks 1–9. Week 1 marked **CURRENT SPRINT**. No daily lessons, no C++ snippets. |
| `quarter-01-week-01-plan.md` | This week’s work: goals, daily micro-lessons (Days 1–7), RAII / smart-pointer snippets. |

Canonical GitHub: https://github.com/ganapetya/learning-bots-sharing  
Local clone: `/opt/src/learning-bots-sharing` (may lag; if `git pull` fails, read GitHub).

Naming:

```
quarter-<nn>-plan.md
quarter-<nn>-week-<ww>-plan.md
quarter-<nn>-week-<ww>-result.md   ← after that week is complete
```

## Quarter 1 (from `quarter-01-plan.md`)

1. **The Coordinate & Memory Foundation** ← this week  
2. Rotations in 3D Space (SO(3) and quaternions)  
3. Camera extrinsics & camera matrix projection  
4. Multi-threaded image acquisition & ring buffers  
5. OpenCV image segmentation & color-space invariance  
6. Deep learning edge deployment (ONNX Runtime on Jetson)  
7. Cat detection & multi-threaded IPC  
8. Behavioral trees for multi-agent tracking (Masha meets Saveli)  
9. Phase 1 capstone: autonomous cat spotting & tracking  

## Week 1 (from `quarter-01-week-01-plan.md`)

- **Math:** 3D frames, basis vectors, meaning of SO(3).  
- **C++:** JVM heap → RAII; `unique_ptr` vs `shared_ptr`.  
- **Exercise:** small offline CMake project; spatial transforms and smart-pointer hand-offs.  
- **Days:** (1) frames, (2) SO(3), (3) stack/heap vs JVM, (4) `unique_ptr`, (5) `shared_ptr`, (6) CMake + gtest, (7) Point3D in `unique_ptr`, move, confirm destruction.

Not started in code on this recognition turn.

## When Week 1 is complete

Write what was done as:

`/opt/src/learning-bots-sharing/quarter-01-week-01-result.md`

Commit and push after Peter authorizes. Do **not** create that file until the week is complete.

## Related

- Repo memo: [`LEARNING_BOTS_SHARING.md`](LEARNING_BOTS_SHARING.md)
- Cooperation v2: [`AI_ROBOTICS_TRIO_COOPERATION.md`](AI_ROBOTICS_TRIO_COOPERATION.md)
