#!/usr/bin/env bash
# Rebuild from source and fail on any missing artifact or acceptance failure.
set -eo pipefail
AMR_PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AMR_WORKSPACE_DIR="$(cd -- "$AMR_PACKAGE_DIR/../.." && pwd)"
AMR_RUNTIME=true
AMR_RENDERS=true
for argument in "$@"; do
  case "$argument" in
    --skip-runtime) AMR_RUNTIME=false ;;
    --no-renders) AMR_RENDERS=false ;;
    *) echo "Usage: bash scripts/build_amr.sh [--skip-runtime] [--no-renders]"; exit 2 ;;
  esac
done
source /opt/ros/jazzy/setup.bash
mkdir -p "$AMR_PACKAGE_DIR/validation"
python3 "$AMR_PACKAGE_DIR/scripts/amr_parameters.py" > "$AMR_PACKAGE_DIR/validation/parameters.json"
blender --background --factory-startup --python-exit-code 1 \
  --python "$AMR_PACKAGE_DIR/scripts/blender/generate_amr.py" -- --no-renders \
  2>&1 | tee "$AMR_PACKAGE_DIR/validation/blender.log"
blender --background --factory-startup --python-exit-code 1 \
  --python "$AMR_PACKAGE_DIR/scripts/blender/test_rebuild.py" \
  > "$AMR_PACKAGE_DIR/validation/rebuild.log" 2>&1
python3 "$AMR_PACKAGE_DIR/scripts/validate_urdf.py"
cd "$AMR_WORKSPACE_DIR"
colcon build --symlink-install 2>&1 | tee "$AMR_PACKAGE_DIR/validation/build.log"
source "$AMR_WORKSPACE_DIR/install/setup.bash"
if "$AMR_RUNTIME"; then
  python3 "$AMR_PACKAGE_DIR/scripts/smoke_test.py" 2>&1 | tee "$AMR_PACKAGE_DIR/validation/acceptance.log"
  python3 "$AMR_PACKAGE_DIR/scripts/validate_gazebo_gui.py"
fi
if "$AMR_RENDERS"; then
  blender --background "$AMR_PACKAGE_DIR/blender/warehouse_amr.blend" --python-exit-code 1 \
    --python "$AMR_PACKAGE_DIR/scripts/blender/render_amr.py" 2>&1 | tee "$AMR_PACKAGE_DIR/validation/renders.log"
fi
echo "AMR rebuilt: $AMR_PACKAGE_DIR (runtime=$AMR_RUNTIME, renders=$AMR_RENDERS)"
