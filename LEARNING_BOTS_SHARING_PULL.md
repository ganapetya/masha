# How grok-build pulls `learning-bots-sharing`

**Clone:** `/opt/src/learning-bots-sharing`  
**Remote:** `git@github.com-learning-bots:ganapetya/learning-bots-sharing.git`  
**Saved:** 2026-08-27  
**Copies:** `~/LEARNING_BOTS_SHARING_PULL.md` and `~/.grok/docs/LEARNING_BOTS_SHARING_PULL.md`  
**Session rule:** `~/.grok/rules/learning-bots-sharing.md` (pull command is there too)

Do **not** put passphrases, key *contents*, or agent secrets in this memo or in chat. Local **file names** are OK.

## Peter’s interactive terminal

From a normal Masha shell (SSH agent / unlocked keys):

```bash
cd /opt/src/learning-bots-sharing
git pull
```

That works. Grok’s command runner is not that shell.

## Grok shell: do not use bare `git pull`

In grok-build, `SSH_AUTH_SOCK` is empty. Bare `git pull` in this clone fails with `Permission denied (publickey)` because the remote host alias `github.com-learning-bots` plus `~/.ssh/id_ed25519_learning_bots` does not finish auth here.

## Command grok-build must use

```bash
cd /opt/src/learning-bots-sharing
GIT_SSH_COMMAND='ssh -i ~/.ssh/id_ed25519_github_robots_world -o IdentitiesOnly=yes -o HostName=github.com' git pull
```

- Key file (name only): `~/.ssh/id_ed25519_github_robots_world`
- `-o HostName=github.com` is required so SSH talks to GitHub, not the alias hostname.
- Verified 2026-08-27: this reports **Already up to date** when the clone matches GitHub `main`.

Same `GIT_SSH_COMMAND` for `git fetch` and, **after Peter authorizes**, `git push`.

## If pull is not needed, HTTPS can still check the tip

```bash
git ls-remote https://github.com/ganapetya/learning-bots-sharing.git HEAD
```

No keys involved. Use this only to compare SHAs; still **pull with the SSH command above** before treating the clone as current.

## Related

Repo use (agreements / plans / results): [`LEARNING_BOTS_SHARING.md`](LEARNING_BOTS_SHARING.md)
