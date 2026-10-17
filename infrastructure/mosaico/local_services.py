#!/usr/bin/env python3
"""Run the official binary and a private PostgreSQL cluster without root.

Tools are installed by bootstrap_native.sh; data lives in ignored .runtime.
Only processes belonging to this workspace are managed. No system services touched.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT/'.runtime/mosaico'
PG = ROOT/'.tools/postgresql/root/usr/lib/postgresql/16/bin'
DAEMON = ROOT/'.tools/mosaico-research/mosaicod'

def env():
    return {**os.environ, 'PGPASSWORD': (DATA/'db-password').read_text().strip(),
        'MOSAICOD_DB_URL': 'postgresql://127.0.0.1:55432/mosaico',
        'MOSAICOD_DB_USER': 'mosaico',
        'MOSAICOD_DB_PASSWORD': (DATA/'db-password').read_text().strip(),
        'MOSAICOD_STORE_ENDPOINT': (DATA/'objects').as_uri(), 'MOSAICOD_STORE_BUCKET': 'telemetry'}

def daemon_pid():
    path = DATA/'mosaicod.pid'
    if path.exists():
        pid = int(path.read_text())
        try:
            if Path(f'/proc/{pid}/exe').resolve() == DAEMON.resolve(): return pid
        except OSError: pass
    return None

def start():
    if subprocess.run([str(PG/'pg_ctl'), '-D', str(DATA/'postgres'), 'status'], stdout=subprocess.DEVNULL).returncode:
        subprocess.run([str(PG/'pg_ctl'), '-D', str(DATA/'postgres'), '-l', str(DATA/'postgres.log'),
            '-o', f'-h 127.0.0.1 -p 55432 -k {DATA}/socket', 'start'], check=True)
    if not daemon_pid():
        with (DATA/'mosaicod.log').open('a') as log:
            proc = subprocess.Popen([str(DAEMON), 'server', '--config', str(Path(__file__).parent/'config.toml'),
                '--host', '127.0.0.1', '--port', '6726', '--log-level', 'info', '--log-format', 'plain'],
                env=env(), stdout=log, stderr=log, start_new_session=True)
        (DATA/'mosaicod.pid').write_text(str(proc.pid))
        time.sleep(1)
        if proc.poll() is not None: raise RuntimeError('mosaicod failed; inspect .runtime/mosaico/mosaicod.log')
    print('PostgreSQL: localhost:55432; mosaicod: localhost:6726')

def stop():
    pid = daemon_pid()
    if pid:
        os.kill(pid, signal.SIGTERM)
        for _ in range(100):
            if not daemon_pid(): break
            time.sleep(.05)
        else: raise RuntimeError('Daemon did not stop; inspect before restarting')
    if (DATA/'postgres/postmaster.pid').exists():
        subprocess.run([str(PG/'pg_ctl'), '-D', str(DATA/'postgres'), '-m', 'fast', 'stop'], check=True)

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('action', choices=['start','stop','restart']); a=p.parse_args()
    if a.action in ('stop','restart'): stop()
    if a.action in ('start','restart'): start()
