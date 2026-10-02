#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

cleanup() {
  set +e
  ./scripts/amsim gui-stop >/dev/null 2>&1 || true
  ./scripts/amsim stop >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[1/8] doctor"
./scripts/amsim doctor

echo "[2/8] desktop/render"
./scripts/amsim desktop-status
./scripts/amsim render-status

echo "[3/8] start runtime"
./scripts/amsim start --wait 75

echo "[4/8] readiness"
./scripts/amsim check

echo "[5/8] attach GUI"
./scripts/amsim gui-start
sleep 3
./scripts/amsim gui-status

echo "[6/8] stop GUI; runtime must survive"
./scripts/amsim gui-stop
./scripts/amsim status

echo "[7/8] stop runtime"
./scripts/amsim stop

echo "[8/8] verify clean shutdown"
if pgrep -af '(^|/)px4( |$)|MicroXRCEAgent|gz sim|gz-sim|amsim.runtime.sitl_supervisor|vglrun' | grep -v 'runtime_visualization_smoke' ; then
  echo "[FAIL] residual simulation/rendering process detected"
  exit 1
fi

echo "[PASS] runtime + readiness + visualization smoke test"
trap - EXIT
