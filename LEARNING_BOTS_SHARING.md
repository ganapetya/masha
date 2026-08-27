# learning-bots-sharing — agreements, plans, and progress

**Local clone on Masha:** `/opt/src/learning-bots-sharing`  
**GitHub:** https://github.com/ganapetya/learning-bots-sharing  
**SSH remote:** `git@github.com-learning-bots:ganapetya/learning-bots-sharing.git`  
**Saved:** 2026-08-27  
**Copies:** `~/LEARNING_BOTS_SHARING.md` and `~/.grok/docs/LEARNING_BOTS_SHARING.md`  
**Session rule:** `~/.grok/rules/learning-bots-sharing.md`

## What this repo is

This is the **shared workspace between Peter, grok-build, and Gemini Notebook**. It is **not** Masha’s robot product repo (`~/ros2_ws` / `ganapetya/masha`).

From `/opt/src/learning-bots-sharing`, grok-build:

1. **Reads our agreements** (including `cooperation-framework-grok-notebook-v2.md`).
2. **Reads our plans** (Sprint Plans, learning logs, memory snippets).
3. **Submits progress files** (sprint progress reports, sprint `src/`, `tests/`) after Peter authorizes the commit and push.

Prefer this tree over chat recap when checking what was decided or how far we got.

## Layout grok-build writes

```
/opt/src/learning-bots-sharing/
├── cooperation-framework-grok-notebook-v2.md
├── learning-log-and-memory-snippets.md
├── sprint-<number>-<start-date>/
│   ├── sprint-<number>-progress-report.md
│   ├── src/
│   └── tests/
└── README.md
```

## How grok-build uses it

- **Read first.** Before a sprint or when Peter refers to an agreement/plan, look in `/opt/src/learning-bots-sharing` (pull if the remote has moved on).
- **Write progress here.** Put sprint reports and the sprint’s implementation/tests in the sprint folder above. Do not put Masha robot runtime code here except as the sprint `src/` the plan asks for.
- **Push only with Peter’s OK.** Gemini Notebook has no GitHub write access. Commit and `git push` from this clone after Peter authorizes.
- **Not the paste drop-box.** Host clipboard/files go through `~/host-clipboard.txt` — see [`HOST_CLIPBOARD_TRANSPORT.md`](HOST_CLIPBOARD_TRANSPORT.md).

## Related

- Cooperation agreement (v2): [`AI_ROBOTICS_TRIO_COOPERATION.md`](AI_ROBOTICS_TRIO_COOPERATION.md)
- Host clipboard transport: [`HOST_CLIPBOARD_TRANSPORT.md`](HOST_CLIPBOARD_TRANSPORT.md)
- Guru notebook (study/review UI, not this git tree): https://notebook.google.com/notebook/84ec42af-0952-438f-bf1e-0decaa1dd52f
