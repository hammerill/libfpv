#!/usr/bin/env bash
# Builds and runs the test suite locally, without network access.
#   tools/run-tests.sh                      native build, every test
#   tools/run-tests.sh sim_options,gps      only these tests (comma-separated)
#   tools/run-tests.sh --web                the same build and run as CI
#                                           (Emscripten + Node, .github/workflows/web.yml)
#   tools/run-tests.sh --repeat 20 sim_options
#                                           run 20 times, count the failures
# Build directories: build/test-native and build/test-web (default options,
# the same as CI). --web needs an Emscripten SDK that is already installed:
# $EMSDK, or ~/emsdk. Installing it is the only step that needs the network:
#   git clone https://github.com/emscripten-core/emsdk.git ~/emsdk
#   ~/emsdk/emsdk install <version> && ~/emsdk/emsdk activate <version>
# (<version> is the one pinned in .github/workflows/web.yml.)
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

web=0
repeat=1
filter=""
while [ $# -gt 0 ]; do
  case "$1" in
    --web) web=1 ;;
    --repeat) repeat="${2:?--repeat needs a count}"; shift ;;
    -h|--help) sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "unknown option $1 (see --help)" >&2; exit 2 ;;
    *) filter="$1" ;;
  esac
  shift
done

if [ ! -f third_party/SimITL/CMakeLists.txt ]; then
  echo "submodules are missing: run git submodule update --init" >&2
  exit 1
fi

if [ "$web" = 1 ]; then
  pinned="$(sed -n 's/.*emsdk install \([0-9.]*\).*/\1/p' .github/workflows/web.yml | head -1)"
  emsdk="${EMSDK:-$HOME/emsdk}"
  if [ ! -f "$emsdk/emsdk_env.sh" ]; then
    echo "no Emscripten SDK at $emsdk (set EMSDK or install it once, online):" >&2
    echo "  git clone https://github.com/emscripten-core/emsdk.git ~/emsdk" >&2
    echo "  ~/emsdk/emsdk install $pinned && ~/emsdk/emsdk activate $pinned" >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  EMSDK_QUIET=1 source "$emsdk/emsdk_env.sh"
  have="$(emcc --version | head -1 | sed -n 's/.* \([0-9][0-9.]*\) (.*/\1/p')"
  if [ -n "$pinned" ] && [ "$have" != "$pinned" ]; then
    echo "note: Emscripten $have here, CI pins $pinned" >&2
  fi
  dir=build/test-web
  emcmake cmake -S . -B "$dir" \
    -DLIBFPV_STANDALONE_BF_CONFIG="$root/assets/configs/hammerill.diff.txt"
  cmake --build "$dir" -j --target fpv_tests
  run() { node "$dir/tests/fpv_tests.js" ${filter:+"$filter"}; }
else
  dir=build/test-native
  cmake -S . -B "$dir"
  cmake --build "$dir" -j --target fpv_tests
  run() { (cd "$dir/tests" && ./fpv_tests ${filter:+"$filter"}); }
fi

if [ "$repeat" -le 1 ]; then
  run
  exit
fi
failed=0
for i in $(seq 1 "$repeat"); do
  echo "=== run $i/$repeat"
  run || failed=$((failed + 1))
done
echo "=== $failed of $repeat runs failed"
[ "$failed" = 0 ]
