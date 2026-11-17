#!/usr/bin/env bash
# Captures the uncommitted changes of a patched submodule that are not yet in
# its patch series as a new patch file.
#   tools/new-patch.sh simitl 0003-short-name
#   tools/new-patch.sh betaflight 0001-short-name
# The submodule working tree is expected to contain all existing patches
# plus the new changes. git writes the diff, so the patch keeps the line
# endings stored in the repository, also where core.autocrlf=true gives the
# working tree CRLF lines (a CRLF patch does not apply to an LF checkout).
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
name="$1"; title="$2"
case "$name" in
  simitl) sub="$root/third_party/SimITL" ;;
  betaflight) sub="$root/third_party/betaflight" ;;
  *) echo "unknown submodule $name" >&2; exit 1 ;;
esac
series="$root/patches/$name"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
# A private index holds HEAD plus the series; the submodule's own index is
# not touched. Untracked files go in as intent-to-add, so they diff as new.
export GIT_INDEX_FILE="$tmp/index"
index_with_series() {
  rm -f "$GIT_INDEX_FILE"
  git -C "$sub" read-tree HEAD
  for p in "$series"/*.patch; do
    [ -e "$p" ] || continue
    git -C "$sub" apply --cached --whitespace=nowarn "$p"
  done
  git -C "$sub" add -N .
}
index_with_series
out="$series/$title.patch"
git -C "$sub" diff --no-color --no-ext-diff --no-renames --src-prefix=a/ --dst-prefix=b/ > "$out"
if [ ! -s "$out" ]; then rm "$out"; echo "no new changes"; exit 0; fi
# verify: the whole series, now with the new patch, must reproduce the working tree
index_with_series
git -C "$sub" diff --quiet || { echo "verification failed: the series does not reproduce the working tree" >&2; exit 1; }
echo "wrote $out"
