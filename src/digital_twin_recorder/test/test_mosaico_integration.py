"""Real Mosaico round trip of every record kind through the storage worker (opt-in).

Runs only with MOSAICO_TEST_HOST set (e.g. 127.0.0.1 with the Compose services up). Creates one
sequence named ``selftest_recorder_<UTC>_<uuid>`` (outside the ``digital_twin_`` listing prefix)
and never deletes anything.
"""
from datetime import datetime, timezone
import os
import time
from uuid import uuid4
import pytest
from digital_twin_recorder.config import Config
from digital_twin_recorder.models.records import convert_message
from digital_twin_recorder.models.recording_metadata import create_metadata
from digital_twin_recorder.storage.session import RecordingSession
from digital_twin_recorder.topics import inventory
from test_inventory import messages
from test_records import odom
from sensor_msgs.msg import Imu

HOST = os.environ.get('MOSAICO_TEST_HOST')


@pytest.mark.skipif(not HOST, reason='set MOSAICO_TEST_HOST to run against a real Mosaico service')
def test_every_kind_round_trips_through_mosaico(tmp_path):
    config = Config(host=HOST, port=int(os.environ.get('MOSAICO_TEST_PORT', '6726')), profile='extended', output=tmp_path,
                    finalize_timeout=float(os.environ.get('MOSAICO_TEST_FINALIZE', '120')))
    specs = inventory(config)
    metadata = create_metadata(config, specs, {'unavailable': ['simulation_session_id']})
    metadata['execution_id'] = f'selftest_recorder_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{uuid4().hex[:8]}'
    session = RecordingSession(config, metadata)
    session.start()
    deadline = time.monotonic()+20
    while session.poll() == 'connecting' and time.monotonic() < deadline:
        time.sleep(.05)
    assert session.state == 'recording', session.error
    samples = {**messages(), 'odometry': odom(), 'imu': Imu()}
    for i in range(3):
        for spec in specs:
            record = convert_message(spec.kind, samples[spec.kind], spec.topic, config.robot_id, received_wall_ns=time.time_ns(),
                                     reception_sim_ns=1_000_000_000*(i+1), tf_pairs=config.tf_pairs)
            record['source_timestamp_ns'] += i            # monotonic per topic
            assert session.submit(record), session.error
            time.sleep(.001)
    report = session.finish('test')
    assert report['state'] == 'completed', report['error']
    v = report['verification']
    assert v['validated'] and v['topics'] == report['topics']
    assert v['topics']['/scan']['last']['ranges'][1] == '+Inf'
    assert v['topics']['/lite_sim/world_state']['last']['entities'][2]['id'] == 'EMP-1'
