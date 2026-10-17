import time
from pathlib import Path
from digital_twin_recorder.config import Config
from digital_twin_recorder.storage.session import RecordingSession
from digital_twin_recorder.models.records import convert
from nav_msgs.msg import Odometry

# Deliberately stalled worker verifies the ROS-side hard bound independently of gRPC.
def stalled_worker(config,metadata,records,result,heartbeat):
    result.put({'state':'recording'})
    time.sleep(30)

def session(tmp_path,**kwargs):
    c=Config(output=tmp_path,queue_size=1,io_timeout=1,finalize_timeout=1,**kwargs)
    return RecordingSession(c,{'execution_id':'unit_test','topics':{'/odom':'nav_msgs/msg/Odometry'}},worker=stalled_worker)

def ready(s):
    deadline=time.monotonic()+5
    while s.poll()=='connecting' and time.monotonic()<deadline: time.sleep(.02)
    assert s.state=='recording'

def test_full_queue_fails_without_blocking(tmp_path):
    s=session(tmp_path); s.start()
    try:
        ready(s); record=convert(Odometry(),'/odom','amr_01')
        assert s.submit(record)
        start=time.monotonic(); assert not s.submit(record)
        assert time.monotonic()-start < .1
        assert s.state=='failed' and 'queue full' in s.error
    finally: report=s.finish()
    assert report['state']=='failed' and not s.process.is_alive()

def test_storage_deadline_kills_stalled_process(tmp_path):
    s=session(tmp_path); s.start()
    try:
        ready(s); s.heartbeat.value=time.monotonic()-2
        assert s.poll()=='failed'; assert 'deadline' in s.error
    finally: s.finish()
    assert not s.process.is_alive()

def test_finalization_has_deadline_and_stops_accepting(tmp_path):
    s=session(tmp_path); s.start(); ready(s)
    start=time.monotonic(); report=s.finish()
    assert time.monotonic()-start < 5
    assert report['state']=='failed'; assert not s.submit({})
    assert s.finish()['state']=='failed'

def test_late_worker_event_cannot_erase_failure(tmp_path):
    from queue import Queue
    s=session(tmp_path)
    s.results.close()
    s.results=Queue()
    s.fail('Source clock reset')
    s.results.put({'state':'completed','verification':{'validated':True}})
    assert s.poll()=='failed'
    assert s.error=='Source clock reset'
    assert s.verification is None
    s.records.close()
