"""The class mapping (configs/dataset.yaml): one source for every stage of the pipeline."""
from functools import lru_cache
from pathlib import Path
import yaml
from .paths import VISION


@lru_cache
def names(path=None):
    data = yaml.safe_load(Path(path or VISION/'configs/dataset.yaml').read_text())
    names = data['names']
    return [names[i] for i in sorted(names)] if isinstance(names, dict) else list(names)


def class_id(kind, path=None):
    return names(path).index(kind)
