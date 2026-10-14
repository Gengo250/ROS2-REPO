# Digital Twin Gateway

Read-only ROS 2 Jazzy telemetry via FastAPI. See
[development](../../docs/digital-twin/DEVELOPMENT.md),
[architecture](../../docs/digital-twin/ARCHITECTURE.md) and
[protocol](../../docs/digital-twin/COMMUNICATION_PROTOCOL.md).

Build with the documented isolated Python 3.12 environment so the ROS executable
uses its dependencies. Start with `ros2 run digital_twin_gateway gateway`, or
`ros2 launch digital_twin_gateway digital_twin.launch.py scenario:=normal`.
No browser is required; missing telemetry produces null poses and availability
states. All interfaces are read-only and bind to 127.0.0.1 by default.
