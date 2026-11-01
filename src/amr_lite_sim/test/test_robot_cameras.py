import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/export_robot_cameras.py'
spec = importlib.util.spec_from_file_location('export_robot_cameras', SCRIPT)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


@pytest.fixture(scope='module')
def described():
    from ament_index_python.packages import get_package_share_directory
    from amr_lite_sim.robot_model import robot_urdf
    share = Path(get_package_share_directory('amr_description'))
    return exporter.describe(robot_urdf(), json.loads((share/'config/amr_dimensions.json').read_text()),
                             yaml.safe_load((share/'config/bridge.yaml').read_text()))


def test_browser_copy_matches_the_robot_model(described):
    # The browser's onboard camera is placed from this file: it must be the URDF's, not a restatement.
    assert json.loads(exporter.OUTPUT.read_text()) == described


def test_optical_frame_convention(described):
    from amr_lite_sim.robot_model import robot_urdf
    position, rotation = exporter.frame_pose(robot_urdf(), 'camera_optical_frame')
    # REP 103 optical frame on a forward-looking camera: z forward (+x base), x right (−y), y down (−z).
    assert np.allclose(rotation@[0, 0, 1], [1, 0, 0]) and np.allclose(rotation@[1, 0, 0], [0, -1, 0])
    assert np.allclose(rotation@[0, 1, 0], [0, 0, -1])
    cam = described['cameras'][0]
    x, y, z, w = cam['quaternion_xyzw']
    q = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    assert np.allclose(q, rotation, atol=1e-6) and np.allclose(cam['position'], position, atol=1e-6)
    # The lens is at the front of the chassis (0.95 m long, base_link at its centre), above the bumper.
    assert .45 < position[0] < .5 and abs(position[1]) < 1e-9 and .2 < position[2] < .3
