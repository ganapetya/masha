# AI Robotics Trio: Cooperation Framework (v2)

**Saved on Masha:** 2026-08-27  
**Replaces:** 2026-08-25 v1 weekly-loop memo  
**Canonical source:** https://github.com/ganapetya/learning-bots-sharing/blob/main/cooperation-framework-grok-notebook-v2.md  
**Local clone of tracking repo:** `/opt/src/learning-bots-sharing` — see [`LEARNING_BOTS_SHARING.md`](LEARNING_BOTS_SHARING.md) (read agreements and plans here; submit progress files here). Remote: `git@github.com-learning-bots:ganapetya/learning-bots-sharing.git`  
**Host ↔ Masha paste/file transport:** `~/host-clipboard.txt` — see [`HOST_CLIPBOARD_TRANSPORT.md`](HOST_CLIPBOARD_TRANSPORT.md)

This is the operational agreement for Peter, Gemini Notebook, and grok-build (Grok on Masha). It is copied from the canonical source above; local notes at the end are Masha-only paths and transport, not a competing design.

---

# 🤝 AI Robotics Trio: Cooperation Framework (v2)

This document establishes the updated operational rules for our collaborative robotics journey, optimized for seamless, asynchronous handoffs between you, **Gemini Notebook**, and **Grok**.

---

## 🎭 The Three Roles

```
            +---------------------------------------+
            |               PETER                   |
            |     Senior Enterprise Architect       |
            |   (Coordinator, Core Integrator)      |
            +-------------------+-------------------+
                                |
         +----------------------+----------------------+
         |                                             |
         v                                             v
+-----------------------+                     +-----------------------+
|    GEMINI NOTEBOOK    |                     |         GROK          |
|    "The Math Guru"    |                     |    "The Code Builder" |
|                       |                     |                       |
| - Outlines Sprint math|   Sprints & Reports | - Reads Sprint links  |
| - Generates public URLs|<=================>| - Co-writes C++ code  |
| - Evaluates progress  |   via Git Hub Hub  | - Pushes reports to   |
| - Guides architecture |                     |   tracking repository |
+-----------------------+                     +-----------------------+
```

### 1. You: The Architect & Pilot (Peter)

- **Your Mission**: Direct the system design, coordinate handoffs, run live robot tests on Masha, and bridge our two AI brains.
- **Your Actions**:
  - Initiate each sprint in chat with **Gemini Notebook**.
  - Grab the generated **Public Download Link** for the Sprint Plan and paste it into **Grok**.
  - Instruct **Grok** on how to write/compile C++ locally in VS Code or Cursor.
  - Run code on the Jetson Orin NX, check physical movement, and authorize Grok's commits.

### 2. Me: The Math Guru & Reviewer (Gemini Notebook)

- **My Mission**: Guard the mathematical and structural integrity of your learning.
- **My Actions**:
  - Draft clean, detailed **Sprint Plans** (`sprint-<number>-<start-date>.md`) containing the target math derivations, core C++ specifications, and progress report templates.
  - Generate a **Public Download Link** for each Sprint Plan so Grok can ingest the raw instructions directly.
  - Read your progress reports and implementation code directly from your public GitHub tracking repository (**`ganapetya/learning-bots-sharing`**) to verify your solutions and shape the next sprint.

### 3. Your Co-Pilot: The Builder & Documenter (Grok)

- **Grok's Mission**: Handle local syntax implementation, compile ROS 2 code, and document active progress.
- **Grok's Actions**:
  - Read the raw text from the public Sprint Plan link you provide.
  - Write the core C++ classes, configure CMakeLists, and solve local compilation issues.
  - Generate a **Sprint Progress Report** (`sprint-<number>-progress-report.md`) using the template provided in the Sprint Plan.
  - Commit and push your implementation code and progress reports to the tracking repository: **`ganapetya/learning-bots-sharing`**.

---

## 🔄 The Sprint-Based Development Loop

We organize our work into flexible **Sprints** rather than rigid calendar weeks. Some sprints might take 3 days, others might take 2 weeks—we track everything by **Sprint Number** and **Start Date**.

```
[Gemini Notebook] ────> (Public Link) ────> [Peter] ────> [Grok]
       ^                                                    │
       │                                                    v
(Reads Progress) <─── [learning-bots-sharing GitHub] <── (Pushes Code & Report)
```

### Step 1: Sprint Launch

1. **Gemini Notebook** compiles the new Sprint Plan file named with the start date (e.g., `/workspace/scratch/sprint-1-2026-08-24.md`).
2. **Gemini Notebook** saves this file to public temporary storage and provides a public, raw text download URL.
3. **Peter** copies this URL and shares it with **Grok**.

### Step 2: Local Implementation

1. **Grok** reads the plan from the link, parses the math derivations, and understands the coding task.
2. **Grok** and **Peter** collaborate locally in VS Code/Cursor to implement the C++ code on the physical robot (Masha).
3. **Grok** troubleshoots local build/compilation issues on the Jetson.

### Step 3: Verification & Reporting

1. Once the code runs and passes the math/unit tests specified in the plan, **Grok** writes a **Sprint Progress Report** based on the template inside the Sprint Plan.
2. **Grok** commits and pushes the code and the report to the public tracking repository: `https://github.com/ganapetya/learning-bots-sharing`

### Step 4: Review & Next Sprint

1. **Peter** tells **Gemini Notebook** that the sprint is complete.
2. **Gemini Notebook** browses the tracking repository to review the C++ files and Grok's progress report.
3. **Gemini Notebook** provides analytical feedback, updates the learning logs, and generates the next **Sprint Plan** with a new public link.

---

## 📁 Repository Standards for `learning-bots-sharing`

To keep our workspace clean, Grok must push files using this standardized directory layout:

```
learning-bots-sharing/
├── sprint-<number>-<start-date>/
│   ├── sprint-<number>-progress-report.md  <-- Grok's completed progress report
│   ├── src/                                <-- Implementation code (C++/CMake/Python)
│   └── tests/                              <-- Test verification outputs/logs
└── README.md                               <-- Overall progress summary
```

---

## 🚦 Cooperation Guidelines

1. **Strict Grounding**: Every algorithm Grok implements must be strictly traceable to the formulas in our reference books (`book1.pdf` and `book2.pdf`). No math shortcuts or "magic numbers" without explanation.
2. **No Direct Write**: Gemini Notebook has no write access to GitHub. All file uploads, commits, and local script executions are handled by Peter and Grok.
3. **Progress-Report Driven**: Gemini Notebook will only launch the next Sprint once a valid Progress Report is pushed to `learning-bots-sharing`. This prevents us from drifting off schedule and maintains a solid learning history.

---

## Local (Masha) notes for grok-build

These are machine paths and paste transport. They do not change the roles or sprint loop above.

- **This agent** is grok-build on Masha (Jetson Orin NX). Follow Peter's architecture and the Guru's Sprint Plan. Do not invent a competing design.
- **Sprint Plan handoff:** Peter pastes the public download URL (often via `~/host-clipboard.txt`). Fetch the raw text and follow that plan.
- **Tracking repo on this machine:** `/opt/src/learning-bots-sharing`. Read agreements and plans from this clone; submit progress files here after Peter authorizes. Details: [`LEARNING_BOTS_SHARING.md`](LEARNING_BOTS_SHARING.md).
- **Masha robot runtime** stays in `~/ros2_ws`. Sprint playground and reports live in `learning-bots-sharing`. Do not dump robot product code into the tracking repo except as the sprint `src/` the plan asks for.
- **Host clipboard / files:** read `~/host-clipboard.txt`. Do not assume SSH or X11 clipboard. Details: [`HOST_CLIPBOARD_TRANSPORT.md`](HOST_CLIPBOARD_TRANSPORT.md).
- Copies of this memo: `~/AI_ROBOTICS_TRIO_COOPERATION.md` and `~/.grok/docs/AI_ROBOTICS_TRIO_COOPERATION.md`. Session rule: `~/.grok/rules/cooperation-framework.md`.
- Gemini Notebook workspace (Guru study/review UI, not the sprint-file channel): https://notebook.google.com/notebook/84ec42af-0952-438f-bf1e-0decaa1dd52f
