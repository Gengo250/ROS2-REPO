"""Checkpoint selection and device resolution shared by evaluation, prediction and the service."""
import hashlib
from pathlib import Path
from .paths import VISION


def resolve_checkpoint(path):
    """``runs/results/latest/...`` → the newest run directory; relative paths are under vision/."""
    p = Path(path)
    if not p.is_absolute():
        p = VISION/p
    parts = list(p.parts)
    if 'latest' in parts:
        i = parts.index('latest')
        root = Path(*parts[:i])
        runs = sorted(r for r in root.iterdir() if r.is_dir() and (r/'checkpoints').is_dir())
        if not runs:
            raise FileNotFoundError(f'no trained run in {root}')
        p = runs[-1].joinpath(*parts[i+1:])
    if not p.is_file():
        raise FileNotFoundError(p)
    return p


def model_identity(path):
    """Stable identifier of a checkpoint: run name + file + short content hash."""
    p = Path(path)
    digest = hashlib.sha256(p.read_bytes()).hexdigest()[:12]
    run = p.parent.parent.name if p.parent.name == 'checkpoints' else p.parent.name
    return {'id': f'{run}/{p.name}@{digest}', 'run': run, 'file': str(p), 'sha256_12': digest}


def resolve_device(value='auto'):
    import torch
    if value == 'auto':
        return 'cuda:0' if torch.cuda.is_available() else 'cpu'
    return value
