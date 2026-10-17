#!/usr/bin/env python3
"""Regenerate the public schemas; --check detects uncommitted contract drift."""
import argparse
import json
from pathlib import Path
from digital_twin_gateway.scene.models import WarehouseScene
from digital_twin_gateway.state.models import ConnectionStatus, TwinState


def schema(model, name):
    return {'$schema': 'https://json-schema.org/draft/2020-12/schema',
            '$id': f'https://digital-twin.local/schemas/{name}.schema.json',
            **model.model_json_schema(mode='serialization')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3] / 'shared/schemas'
    for model, name in ((TwinState, 'twin-state'), (ConnectionStatus, 'connection-status'), (WarehouseScene, 'warehouse-scene')):
        path = root / f'{name}.schema.json'
        text = json.dumps(schema(model, name), indent=2) + '\n'
        if args.check:
            if path.read_text() != text:
                raise SystemExit(f'Schema drift: regenerate {path}')
        else:
            path.write_text(text)


if __name__ == '__main__':
    main()
