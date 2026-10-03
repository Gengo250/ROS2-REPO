#!/usr/bin/env bash
# No deletion, online assets, Blender or modification of AMR dimensions.
set -eo pipefail
WAREHOUSE_PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WAREHOUSE_WORKSPACE_DIR="$(cd -- "$WAREHOUSE_PACKAGE_DIR/../.." && pwd)"
WAREHOUSE_RUNTIME=true
for argument in "$@"; do
  case "$argument" in
    --skip-runtime) WAREHOUSE_RUNTIME=false ;;
    *) echo "Usage: bash scripts/build_warehouse.sh [--skip-runtime]"; exit 2 ;;
  esac
done
source /opt/ros/jazzy/setup.bash
python3 "$WAREHOUSE_PACKAGE_DIR/scripts/generate_warehouse.py"
python3 "$WAREHOUSE_PACKAGE_DIR/scripts/validate_warehouse.py"
cd "$WAREHOUSE_WORKSPACE_DIR"
colcon build 2>&1 | tee "$WAREHOUSE_PACKAGE_DIR/validation/build.txt"
source install/setup.bash
colcon test --event-handlers console_direct+ 2>&1 | tee "$WAREHOUSE_PACKAGE_DIR/validation/workspace_tests.txt"
if ! colcon test-result --verbose > "$WAREHOUSE_PACKAGE_DIR/validation/workspace_test_results.txt" 2>&1; then
  cat "$WAREHOUSE_PACKAGE_DIR/validation/workspace_test_results.txt"
  echo "Workspace contains test failures; see workspace_test_results.txt (baseline: my_py_pkg flake8)."
fi
# Fail on warehouse regressions even when unrelated examples have baseline failures.
colcon test-result --test-result-base build/amr_simulation --verbose
if "$WAREHOUSE_RUNTIME"; then
  python3 "$WAREHOUSE_PACKAGE_DIR/scripts/validate_amr_regression.py"
  python3 "$WAREHOUSE_PACKAGE_DIR/scripts/validate_scenario_gui.py"
  python3 "$WAREHOUSE_PACKAGE_DIR/scripts/run_acceptance.py"
  python3 "$WAREHOUSE_PACKAGE_DIR/scripts/validate_repeatability.py"
  python3 "$WAREHOUSE_PACKAGE_DIR/scripts/report_warehouse.py"
fi
echo "Warehouse build and acceptance passed (runtime=$WAREHOUSE_RUNTIME); workspace results recorded separately."
