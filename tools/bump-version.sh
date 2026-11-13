#!/usr/bin/env bash
# Bumps libfpv's version in include/libfpv.h, the only place it is set
# (CMakeLists.txt reads it from there).
#   tools/bump-version.sh patch    # 0.1.0 -> 0.1.1
#   tools/bump-version.sh minor    # 0.1.3 -> 0.2.0
#   tools/bump-version.sh major    # 0.2.1 -> 1.0.0
#   tools/bump-version.sh 1.2.3    # set it outright
#   tools/bump-version.sh          # print the current version
# Leaves the change in the working tree; nothing is committed.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
header="$root/include/libfpv.h"

read_part() {
  local v
  v="$(sed -nE "s/^#define LIBFPV_VERSION_$1 +([0-9]+)$/\1/p" "$header")"
  [[ "$v" =~ ^[0-9]+$ ]] || { echo "can't read LIBFPV_VERSION_$1 from $header" >&2; exit 1; }
  echo "$v"
}
major="$(read_part MAJOR)"; minor="$(read_part MINOR)"; patch="$(read_part PATCH)"
old="$major.$minor.$patch"

case "${1:-}" in
  "") echo "$old"; exit 0 ;;
  major) major=$((major + 1)); minor=0; patch=0 ;;
  minor) minor=$((minor + 1)); patch=0 ;;
  patch) patch=$((patch + 1)) ;;
  *)
    if [[ "$1" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
      major=$((10#${BASH_REMATCH[1]})); minor=$((10#${BASH_REMATCH[2]})); patch=$((10#${BASH_REMATCH[3]}))
    else
      echo "usage: $0 [major|minor|patch|X.Y.Z]" >&2; exit 1
    fi ;;
esac
new="$major.$minor.$patch"
[[ "$new" != "$old" ]] || { echo "already $old" >&2; exit 1; }

perl -pi -e "
  s/^(#define LIBFPV_VERSION_MAJOR +)[0-9]+\$/\${1}$major/;
  s/^(#define LIBFPV_VERSION_MINOR +)[0-9]+\$/\${1}$minor/;
  s/^(#define LIBFPV_VERSION_PATCH +)[0-9]+\$/\${1}$patch/;
" "$header"

[[ "$(read_part MAJOR).$(read_part MINOR).$(read_part PATCH)" == "$new" ]] ||
  { echo "failed to write $new to $header" >&2; exit 1; }
echo "libfpv $old -> $new"
echo "For an incompatible ABI change, also bump LIBFPV_ABI_VERSION in" \
  "include/libfpv.h and SOVERSION in CMakeLists.txt."
