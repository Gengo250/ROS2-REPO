"""Bounded process/queue orchestration; never network I/O in submit()."""
from datetime import datetime, timezone
import json
import multiprocessing as mp
from queue import Empty, Full
import time
from ..models.records import Integrity
from .worker import write

class RecordingSession:
    def __init__(self, config, metadata, worker=write):
        self.config,self.metadata=config,metadata
        self.state='idle'; self.error=None; self.verification=None
        self.integrity={topic:Integrity() for topic in metadata['topics']}
        self.context=mp.get_context('spawn')
        self.records=self.context.Queue(maxsize=config.queue_size)
        self.results=self.context.Queue(maxsize=4)
        self.heartbeat=self.context.Value('d',time.monotonic())
        self.process=self.context.Process(target=worker,args=(config,metadata,self.records,self.results,self.heartbeat))
        self.path=config.output/metadata['execution_id']; self.path.mkdir(parents=True,exist_ok=False)
        self.started=time.monotonic(); self.finalized=False; self.stop_reason=None

    def report(self):
        return dict(state=self.state,error=self.error,metadata=self.metadata,stop_reason=self.stop_reason,
            elapsed_wall_seconds=time.monotonic()-self.started,
            topics={topic:value.report() for topic,value in self.integrity.items()},verification=self.verification)

    def save(self):
        target=self.path/'report.json'; temp=self.path/'report.tmp'
        temp.write_text(json.dumps(self.report(),indent=2)+'\n'); temp.replace(target)

    def start(self):
        self.state='connecting'; self.process.start(); self.save()

    def poll(self):
        while True:
            try: event=self.results.get_nowait()
            except Empty: break
            self.state=event['state']; self.error=event.get('error'); self.verification=event.get('verification')
        if self.state in ('connecting','recording'):
            if not self.process.is_alive(): self.fail('Storage worker exited unexpectedly')
            elif time.monotonic()-self.heartbeat.value > self.config.io_timeout: self.fail('Storage I/O deadline exceeded')
        return self.state

    def fail(self, reason):
        self.state='failed'; self.error=reason

    def submit(self, record):
        if self.state != 'recording': return False
        try:
            previous=self.integrity[record['source_topic']].last
            if previous and record['source_timestamp_ns'] < previous['source_timestamp_ns']:
                raise ValueError('Source clock reset or out-of-order telemetry; start a new execution')
            self.records.put_nowait(record)
            self.integrity[record['source_topic']].add(record)
            return True
        except (Full,ValueError) as error:
            self.fail(str(error) or 'Recorder queue full; recording failed without blocking ROS')
            return False

    def finish(self, reason='requested_stop'):
        if self.finalized: return self.report()
        self.finalized=True; self.stop_reason=reason
        if self.state == 'recording':
            self.state='finalizing'
            summary=dict(topics={t:v.report() for t,v in self.integrity.items()},
                ended_wall_utc=datetime.now(timezone.utc).isoformat(),stop_reason=reason)
            deadline=time.monotonic()+self.config.finalize_timeout
            try:
                self.records.put({'finalize':summary},timeout=min(2,self.config.finalize_timeout))
                while self.state=='finalizing' and time.monotonic()<deadline:
                    self.poll(); time.sleep(.05)
                if self.state!='completed': self.fail(self.error or 'Finalization deadline exceeded')
            except Full: self.fail('Finalization queue deadline exceeded')
        elif self.state not in ('completed','failed'): self.fail('Stopped before storage became ready')
        self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.terminate(); self.process.join(timeout=2)
            if self.process.is_alive(): self.process.kill(); self.process.join(timeout=2)
        # A full failed queue must not hang Python's multiprocessing feeder at exit.
        self.records.cancel_join_thread(); self.records.close()
        self.results.cancel_join_thread(); self.results.close()
        self.save(); return self.report()
