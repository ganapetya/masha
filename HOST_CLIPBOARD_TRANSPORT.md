# Host ↔ Masha clipboard transport

**Drop-box on Masha:** `/home/ubuntu/host-clipboard.txt` (`~/host-clipboard.txt`)  
**Saved:** 2026-08-27  
**Copies:** `~/HOST_CLIPBOARD_TRANSPORT.md` and `~/.grok/docs/HOST_CLIPBOARD_TRANSPORT.md`  
**Session rule:** `~/.grok/rules/host-clipboard-transport.md`

## What this is

`~/host-clipboard.txt` is the **transport between Peter's host (laptop) and Masha (this Jetson)** for **texts and files**.

It is the usual way to hand Grok a host clipboard paste, a URL, or file contents. **SSH is not always used.** X11 / Wayland clipboard forwarding is often missing (`DISPLAY` empty; `xclip` / `wl-paste` do not see the host clipboard). Do not wait for OSC-52 or a remote X selection.

This file is a live drop-box, not a git artifact. Home gitignore leaves `~/host-clipboard.txt` out of `masha.git`.

## What it carries

Peter (or a host-side copier) writes into `~/host-clipboard.txt`. Contents may be:

- Plain text (an agreement, a prompt, notes)
- A URL (GitHub blob, raw Sprint Plan, gist)
- A host file path, or the file's contents
- Several of the above in one paste

This is **text and file** transport, not a string-only clipboard.

## How grok-build uses it

1. When Peter says he pasted on the **host clipboard**, **read `~/host-clipboard.txt` first**. Do not ask him to re-paste into the TUI unless the file is empty or stale.
2. If the file is a URL, fetch the target (prefer the raw form for GitHub markdown).
3. If it names files or contains file bodies, treat those as the payload.
4. After using it, leave the file as Peter left it unless he asks you to clear or replace it.
5. Do not confuse this drop-box with:
   - the tracking repo `/opt/src/learning-bots-sharing`
   - Masha robot code `~/ros2_ws`
   - Grok's own copy backup `~/.grok/last-copy.txt` (that is *from* Grok, not *from* the host)

## How Peter uses it

Paste host clipboard (or copy a file) into `~/host-clipboard.txt` on Masha, then tell grok-build to read it. No SSH session and no X11 forward required.

Optional SSH/X11 clipboard (`Desktop/host-masha-ssh.config`) still exists; it is not the default path.

## Related

- Cooperation agreement (v2): [`AI_ROBOTICS_TRIO_COOPERATION.md`](AI_ROBOTICS_TRIO_COOPERATION.md)
- Agreements / plans / progress repo: [`LEARNING_BOTS_SHARING.md`](LEARNING_BOTS_SHARING.md)
