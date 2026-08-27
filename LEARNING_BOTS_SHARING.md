# learning-bots-sharing — agreements, plans, and progress

**Local clone on Masha:** `/opt/src/learning-bots-sharing`  
**GitHub:** https://github.com/ganapetya/learning-bots-sharing  
**SSH remote:** `git@github.com-learning-bots:ganapetya/learning-bots-sharing.git`  
**How grok-build pulls:** [`LEARNING_BOTS_SHARING_PULL.md`](LEARNING_BOTS_SHARING_PULL.md) (do not use bare `git pull` from the Grok shell)  
**Saved:** 2026-08-27  
**Copies:** `~/LEARNING_BOTS_SHARING.md` and `~/.grok/docs/LEARNING_BOTS_SHARING.md`  
**Session rule:** `~/.grok/rules/learning-bots-sharing.md`

## What this repo is

This is the **shared workspace between Peter, grok-build, and Gemini Notebook**. It is **not** Masha’s robot product repo (`~/ros2_ws` / `ganapetya/masha`).

From `/opt/src/learning-bots-sharing`, grok-build:

1. **Reads our agreements** (including `cooperation-framework-grok-notebook-v2.md`).
2. **Reads our plans** — **quarter plans** and **week plans** inside a quarter.
3. **Submits progress files** after Peter authorizes the commit and push.

Prefer this tree over chat recap.

**Now (2026-08-27):** Quarter 1, Week 1. See [`CURRENT_PLAN.md`](CURRENT_PLAN.md). When that week is complete, write `quarter-01-week-01-result.md` here (not before).

## Plan vs result naming

```
learning-bots-sharing/
├── cooperation-framework-grok-notebook-v2.md
├── learning-log-and-memory-snippets.md
├── quarter-01-plan.md                 # quarter hub / tracker only
├── quarter-01-week-01-plan.md         # that week's work
├── quarter-01-week-01-result.md       # after Week 1 is complete
└── README.md
```

- **Quarter plan** (`quarter-<nn>-plan.md`): syllabus and progress tracker for the quarter. Not a copy of one week’s daily lessons.
- **Week plan** (`quarter-<nn>-week-<ww>-plan.md`): that week’s goals, lessons, coding task.
- **Week result** (`quarter-<nn>-week-<ww>-result.md`): what grok-build writes when the week is done.

Week implementation (CMake/C++ playground) that the week plan asks for also lands in this repo, next to that week’s files, after Peter asks to implement.

## How grok-build uses it

- **Read first.** Pull with the command in [`LEARNING_BOTS_SHARING_PULL.md`](LEARNING_BOTS_SHARING_PULL.md), then read this clone. Do not use bare `git pull` from Grok.
- **Write results here** using the week-result name above. Do not put Masha robot runtime code here except what a week plan asks to submit.
- **Push only with Peter’s OK.** Gemini Notebook has no GitHub write access.
- **Not the paste drop-box.** Host clipboard/files go through `~/host-clipboard.txt` — see [`HOST_CLIPBOARD_TRANSPORT.md`](HOST_CLIPBOARD_TRANSPORT.md).

## Related

- Current plan: [`CURRENT_PLAN.md`](CURRENT_PLAN.md)
- How grok-build pulls this clone: [`LEARNING_BOTS_SHARING_PULL.md`](LEARNING_BOTS_SHARING_PULL.md)
- Cooperation agreement (v2): [`AI_ROBOTICS_TRIO_COOPERATION.md`](AI_ROBOTICS_TRIO_COOPERATION.md)
- Host clipboard transport: [`HOST_CLIPBOARD_TRANSPORT.md`](HOST_CLIPBOARD_TRANSPORT.md)
