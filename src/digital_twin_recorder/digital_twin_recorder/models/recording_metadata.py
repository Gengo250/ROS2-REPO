from datetime import datetime, timezone
import hashlib
import subprocess
from pathlib import Path
from uuid import uuid4

def create_metadata(config):
    from ament_index_python.packages import get_package_share_directory
    now = datetime.now(timezone.utc)
    execution_id = f'digital_twin_{config.scenario}_{now.strftime("%Y%m%dT%H%M%SZ")}_{uuid4().hex[:12]}'
    paths = {'warehouse_config_sha256':Path(get_package_share_directory('amr_simulation'))/'config/warehouse_layout.json',
             'amr_config_sha256':Path(get_package_share_directory('amr_description'))/'config/amr_dimensions.json'}
    try: commit = subprocess.check_output(['git','rev-parse','HEAD'],text=True,stderr=subprocess.DEVNULL).strip()
    except (subprocess.SubprocessError, OSError): commit = 'unavailable'
    return dict(execution_id=execution_id, project='digital_twin', environment='simulation', world='warehouse',
        scenario=config.scenario, robot_id=config.robot_id, ros_distribution='jazzy', simulator='gazebo_harmonic',
        coordinate_frame='source_message_frame', pose_semantics='local_odometry_not_global_localization',
        recording_source='ros2', schema_version='1.0', started_wall_utc=now.isoformat(),git_commit=commit,
        topics={config.odom_topic:'nav_msgs/msg/Odometry',config.imu_topic:'sensor_msgs/msg/Imu'},
        timestamp_semantics={'source_timestamp_ns':'ROS header simulation time','received_wall_ns':'Unix UTC nanoseconds'},
        **{name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in paths.items()})
