# Twin protocol v1.0

The checked-in JSON Schemas are the public wire contracts. Source models are in
`src/digital_twin_gateway/digital_twin_gateway/state/models.py`. Generate with:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python src/digital_twin_gateway/scripts/export_schemas.py
cd web/digital-twin
npm run generate:types
```

`export_schemas.py --check`, Gateway tests and `npm run check:types` catch drift.
Browser types are generated from these schemas, and Ajv 2020-12 validates every
received state/status at runtime. No hand-maintained duplicate telemetry interface.
All defined properties are required on the wire; missing telemetry uses null.

`humans`, `obstacles`, `stations` are empty reserved arrays in v1.0. Entity payloads
need a defined contract in a later protocol revision. Strict readers reject unknown
properties and unsupported versions; coordinate protocol changes across both ends.
See [communication protocol](../../docs/digital-twin/COMMUNICATION_PROTOCOL.md).
