#!/usr/bin/env bash
# Ubuntu 24.04 amd64 fallback: extract distro PostgreSQL tools without installation.
set -euo pipefail
cd "$(dirname "$0")/../.."
mkdir -p .tools/mosaico-research .tools/postgresql
(
  cd .tools/mosaico-research
  base=https://github.com/mosaico-labs/mosaico/releases/download/v0.7.0
  curl -fL "$base/mosaicod-linux-amd64.tar.gz" -o mosaicod-linux-amd64.tar.gz
  curl -fL "$base/mosaicod-linux-amd64.tar.gz.sha256" -o mosaicod-linux-amd64.tar.gz.sha256
  sha256sum -c mosaicod-linux-amd64.tar.gz.sha256
  tar -xzf mosaicod-linux-amd64.tar.gz
)
(
  cd .tools/postgresql
  apt download postgresql-16=16.15-0ubuntu0.24.04.1 postgresql-client-16=16.15-0ubuntu0.24.04.1 libpq5=16.15-0ubuntu0.24.04.1
  for package in *.deb; do dpkg-deb -x "$package" root; done
)
python3 - <<'PY'
from pathlib import Path
import os, secrets, subprocess
root=Path.cwd(); data=root/'.runtime/mosaico'
for part in ('objects','socket'): (data/part).mkdir(parents=True,exist_ok=True)
password=data/'db-password'
if not password.exists():
    password.write_text(secrets.token_urlsafe(32)+'\n'); password.chmod(0o600)
pg=root/'.tools/postgresql/root/usr/lib/postgresql/16/bin'
if not (data/'postgres/PG_VERSION').exists():
    subprocess.run([str(pg/'initdb'),'-D',str(data/'postgres'),'-L',str(root/'.tools/postgresql/root/usr/share/postgresql/16'),
        '-U','mosaico','--auth=scram-sha-256','--pwfile='+str(password)],check=True)
# PostgreSQL must be ready and database created BEFORE starting mosaicod.
if subprocess.run([str(pg/'pg_ctl'),'-D',str(data/'postgres'),'status'],stdout=subprocess.DEVNULL).returncode:
    subprocess.run([str(pg/'pg_ctl'),'-D',str(data/'postgres'),'-l',str(data/'postgres.log'),
        '-o',f'-h 127.0.0.1 -p 55432 -k {data}/socket','start'],check=True)
env={**os.environ,'PGPASSWORD':password.read_text().strip()}
exists=subprocess.check_output([str(pg/'psql'),'-h','127.0.0.1','-p','55432','-U','mosaico','-d','postgres','-Atc',
    "SELECT 1 FROM pg_database WHERE datname='mosaico'"],env=env,text=True).strip()
if not exists:
    subprocess.run([str(pg/'createdb'),'-h','127.0.0.1','-p','55432','-U','mosaico','mosaico'],env=env,check=True)
PY
python3 infrastructure/mosaico/local_services.py start
