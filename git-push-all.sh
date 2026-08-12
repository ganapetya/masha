#!/usr/bin/env bash
# Add, commit, and push the home-root Masha repo (github.com/ganapetya/masha).
#
# Usage:
#   ./git-push-all.sh                     # commit with a timestamp message, then push
#   ./git-push-all.sh "my message"        # commit with that message, then push
#   ./git-push-all.sh --force "message"   # same, but force-with-lease (overwrites remote)
#
# Nested vendor .git dirs (ros2_ws, software, …) are hidden only during
# `git add` so those files join this monorepo instead of becoming submodules.

set -euo pipefail

ROOT="${HOME}"
KEY="${HOME}/.ssh/id_ed25519_github_robots_world"
export GIT_SSH_COMMAND="ssh -i ${KEY} -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new"

cd "${ROOT}"

if [[ ! -d .git ]]; then
  echo "No git repo in ${ROOT}. Expected git root at ~." >&2
  exit 1
fi

FORCE=0
if [[ "${1:-}" == "--force" || "${1:-}" == "-f" ]]; then
  FORCE=1
  shift
fi

MSG="${*:-update from masha $(date +%Y-%m-%d_%H:%M:%S)}"

# Only look inside trees we actually track.
SEARCH_ROOTS=()
for d in ros2_ws software wifi_manager; do
  [[ -d "${ROOT}/${d}" ]] && SEARCH_ROOTS+=("${ROOT}/${d}")
done

NESTED_GIT_DIRS=()
if ((${#SEARCH_ROOTS[@]})); then
  while IFS= read -r -d '' gitdir; do
    NESTED_GIT_DIRS+=("$gitdir")
  done < <(find "${SEARCH_ROOTS[@]}" -name .git -print0 2>/dev/null)
fi

hide_nested() {
  local d
  for d in "${NESTED_GIT_DIRS[@]+"${NESTED_GIT_DIRS[@]}"}"; do
    if [[ -e "$d" && ! -e "${d}.nested" ]]; then
      mv "$d" "${d}.nested"
    fi
  done
}

restore_nested() {
  local d
  for d in "${NESTED_GIT_DIRS[@]+"${NESTED_GIT_DIRS[@]}"}"; do
    if [[ -e "${d}.nested" ]]; then
      mv "${d}.nested" "$d"
    fi
  done
}

trap restore_nested EXIT
hide_nested

git add -A

if git ls-files -s | awk '$1 == "160000" { found=1; print } END { exit !found }'; then
  echo "Refusing to commit gitlinks/submodules. Nested .git hide failed." >&2
  git ls-files -s | awk '$1 == "160000"'
  exit 1
fi

if git diff --cached --quiet; then
  echo "Nothing new to commit."
else
  git commit -m "${MSG}"
fi

restore_nested
trap - EXIT

if ! git remote get-url origin >/dev/null 2>&1; then
  echo "No 'origin' remote. Add it with:" >&2
  echo "  git remote add origin git@github.com:ganapetya/masha.git" >&2
  exit 1
fi

if [[ ! -f "${KEY}" ]]; then
  echo "SSH key missing: ${KEY}" >&2
  echo "Generate one, then add the .pub file to GitHub → Settings → SSH keys." >&2
  exit 1
fi

BRANCH="$(git rev-parse --abbrev-ref HEAD)"

if ((FORCE)); then
  git push --force-with-lease -u origin "${BRANCH}"
else
  if ! git push -u origin "${BRANCH}"; then
    echo
    echo "Push rejected. If origin only has the placeholder readme, rerun:"
    echo "  ${ROOT}/git-push-all.sh --force"
    echo
    echo "If GitHub says Permission denied, add this public key to"
    echo "GitHub → Settings → SSH and GPG keys → New SSH key:"
    echo "-----"
    cat "${KEY}.pub"
    echo "-----"
    exit 1
  fi
fi

echo "Done: pushed ${BRANCH} to $(git remote get-url origin)"
