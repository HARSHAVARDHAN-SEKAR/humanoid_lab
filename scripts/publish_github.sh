#!/usr/bin/env bash
# Publish HUMANOID-LAB to the user's existing public GitHub repository.
# Runs on the user's Ubuntu host, not in Docker. Never force-pushes.
set -euo pipefail
cd "$(dirname "$0")/.."
repository=HARSHAVARDHAN-SEKAR/humanoid_lab
url="https://github.com/${repository}.git"

command -v git >/dev/null || { echo 'Install git first: sudo apt install git' >&2; exit 2; }

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git init
fi

git add -A
if ! git diff --cached --quiet; then
  git commit -m 'HUMANOID-LAB: supported biped control and ROS Docker fixes'
fi

git branch -M main
if git remote get-url origin >/dev/null 2>&1; then
  remote="$(git remote get-url origin)"
  case "$remote" in
    "$url"|"git@github.com:${repository}.git"|"https://github.com/${repository}"|"git@github.com:${repository}") ;;
    *) echo "Unexpected origin: $remote" >&2; echo "Expected: $url" >&2; exit 2 ;;
  esac
else
  git remote add origin "$url"
fi

echo "Pushing main to $url (no force)."
echo 'If authentication is required: gh auth login && gh auth setup-git'
git push -u origin main
printf 'Published: https://github.com/%s\n' "$repository"
