#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

test "$(git branch --show-current)" = simulation
./scripts/amsim setup --check-only
./scripts/amsim doctor
./scripts/amsim render-status

if grep -RIn   --exclude-dir=legacy   --exclude-dir=runs   --exclude-dir=.git   --exclude-dir=generated   --exclude-dir=__pycache__   '/root/autodl-tmp' src scripts configs deploy/host/*.sh 2>/dev/null; then
  echo "[FAIL] product code contains development-server hard-coded path"
  exit 2
fi

echo "[PASS] portable preflight"
