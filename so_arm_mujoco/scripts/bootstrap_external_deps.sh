#!/usr/bin/env bash
set -euo pipefail

# Reproducible install for so_arm_mujoco's pip-only deps (not rosdep keys - see package.xml).
# Usage:
#   ./so_arm_mujoco/scripts/bootstrap_external_deps.sh [--gym]

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

echo "[1/2] Installing pip dependencies from $PKG_DIR/requirements.txt"
python3 -m pip install -r "$PKG_DIR/requirements.txt" --break-system-packages

if [[ "${1:-}" == "--gym" ]]; then
  echo "[1b/2] Installing Gymnasium extra from $PKG_DIR/requirements-gym.txt"
  python3 -m pip install -r "$PKG_DIR/requirements-gym.txt" --break-system-packages
fi

echo "[2/2] Verifying imports"
python3 - "$@" <<'PY'
import sys
import mujoco
print('OK: imported mujoco', mujoco.__version__)
if '--gym' in sys.argv:
    import gymnasium
    print('OK: imported gymnasium', gymnasium.__version__)
PY

echo "External dependency bootstrap complete."
