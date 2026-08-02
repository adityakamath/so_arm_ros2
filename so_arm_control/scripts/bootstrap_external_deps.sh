#!/usr/bin/env bash
set -euo pipefail

# Reproducible install for external deps used by so_arm_control's collision checker.
# Usage:
#   ./so_arm_control/scripts/bootstrap_external_deps.sh

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
REQ_FILE="$PKG_DIR/requirements.txt"

if [[ ! -f "$REQ_FILE" ]]; then
  echo "requirements file not found: $REQ_FILE" >&2
  exit 1
fi

echo "[1/3] Installing native FCL prerequisite (libfcl-dev)"
sudo apt-get update
sudo apt-get install -y libfcl-dev

echo "[2/3] Installing pip dependencies from $REQ_FILE"
python3 -m pip install -r "$REQ_FILE" --break-system-packages

echo "[3/3] Verifying imports"
python3 - <<'PY'
import fcl
from stl import mesh
print('OK: imported fcl and numpy-stl')
PY

echo "External dependency bootstrap complete."
