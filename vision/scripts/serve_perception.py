#!/usr/bin/env python3
"""Live perception service for the onboard cameras of the lite twin's AMRs.

    source /opt/ros/jazzy/setup.bash && source ../install/setup.bash     # optional: ROS topics
    uv run python scripts/serve_perception.py [--config configs/inference.yaml] [--model runs/results/<id>/checkpoints/best.pt]

Needs the Gateway (lite_twin / lite_navigation launch) and the built web bundle (camera.html).
Starts its own camera render worker (Chromium) and supervises it.
HTTP/WS on 127.0.0.1:8100:
  WS   /ws/v1/perception?robot_id=amr_02   processed frames + detections of one robot (binary:
                                           uint32 LE header length, JSON header, JPEG)
  GET  /api/v1/perception/status           model, device, per-robot FPS, latency, drops, errors
  POST /api/v1/perception/config           {"enabled", "confidence_threshold", "max_inference_fps_per_robot", "model_path"}
  GET  /api/v1/perception/models           trained checkpoints available under runs/results
ROS (when rclpy is importable): <ns>/camera/image_raw (rgb8), <ns>/camera/camera_info,
<ns>/perception/detections (vision_msgs/Detection2DArray), stamped with the frame's simulation time.
"""
import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
import os
import socket
import sys
import time
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen
import yaml
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, ConfigDict, Field

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atlas_vision.paths import VISION  # noqa: E402
from atlas_vision.perception import Perception  # noqa: E402
from atlas_vision.render_worker import launch_browser, mount_worker  # noqa: E402


class ConfigRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    enabled: bool | None = None
    confidence_threshold: float | None = Field(default=None, ge=.01, le=.99)
    max_inference_fps_per_robot: float | None = Field(default=None, gt=0, le=15)
    model_path: str | None = Field(default=None, max_length=512)


def gateway_robots(gateway):
    """AMR ids in the Gateway's current world state."""
    with urlopen(f'{gateway}/api/v1/world', timeout=5) as response:
        world = json.loads(response.read())
    return [e['id'] for e in world['entities'] if e['type'] == 'amr']


def create_app(config, robots):
    perception = Perception(config, robots)
    state = {'browser': None, 'profile': None, 'restarts': 0}

    async def supervise():
        """Start the camera worker once the server listens; restart it if it dies."""
        port = config['port']
        while True:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=1):
                    break
            except OSError:
                await asyncio.sleep(.2)
        query = urlencode({'channel': f'ws://127.0.0.1:{port}/ws/render', 'gateway': config['gateway']})
        while True:
            process = state['browser']
            if process is None or process.poll() is not None:
                if process is not None:
                    state['restarts'] += 1
                try:
                    log = VISION/'runs/perception_worker.log'
                    log.parent.mkdir(parents=True, exist_ok=True)
                    state['browser'], state['profile'] = launch_browser(f'http://127.0.0.1:{port}/camera.html?{query}', config['browser'], str(log))
                except Exception as error:      # keep supervising; the status endpoint shows the reason
                    state['error'] = f'camera worker not started: {error}'
                    print(state['error'], flush=True)
            await asyncio.sleep(2.)

    @asynccontextmanager
    async def lifespan(app):
        perception.start()
        supervisor = asyncio.create_task(supervise())
        try:
            yield
        finally:
            supervisor.cancel()
            with suppress(asyncio.CancelledError):
                await supervisor
            process = state['browser']
            if process is not None and process.poll() is None:
                import os
                with suppress(ProcessLookupError):
                    os.killpg(process.pid, 15)
            await perception.stop()

    app = FastAPI(title='ATLAS perception', version='1.0', lifespan=lifespan)
    app.add_middleware(CORSMiddleware, allow_origins=list(config['allowed_origins']), allow_methods=['GET', 'POST'],
                       allow_headers=['Content-Type'], allow_credentials=False)

    @app.get('/api/v1/perception/status')
    async def status():
        return {**perception.status(), 'worker_restarts': state['restarts'], 'worker_launch_error': state.get('error'),
                'gateway': config['gateway']}

    @app.post('/api/v1/perception/config')
    async def configure(body: ConfigRequest):
        if body.model_path is not None:
            try:
                async with perception.model_lock:
                    await asyncio.get_running_loop().run_in_executor(perception.executor, perception.detector.load, body.model_path)
            except (FileNotFoundError, ValueError) as error:
                raise HTTPException(409, f'model not loaded: {error}') from error
        if body.confidence_threshold is not None:
            perception.detector.confidence = body.confidence_threshold
        if body.max_inference_fps_per_robot is not None:
            perception.fps = body.max_inference_fps_per_robot
        if body.enabled is not None:
            perception.enabled = body.enabled
        return perception.status()

    @app.get('/api/v1/perception/models')
    async def models():
        root = VISION/'runs/results'
        runs = sorted(root.glob('*/checkpoints/*.pt')) if root.is_dir() else []
        return {'current': perception.detector.identity, 'available': [str(p.relative_to(VISION)) for p in runs]}

    @app.websocket('/ws/v1/perception')
    async def stream(socket: WebSocket, robot_id: str):
        await perception.serve_client(socket, robot_id)

    mount_worker(app, perception.channel)
    return app


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', default=str(VISION/'configs/inference.yaml'))
    p.add_argument('--model', help='checkpoint (overrides model_path)')
    p.add_argument('--port', type=int)
    p.add_argument('--browser', choices=['auto', 'nvidia', 'gpu', 'headless'])
    p.add_argument('--no-ros', action='store_true')
    a = p.parse_args()
    import uvicorn
    config = yaml.safe_load(Path(a.config).read_text())
    if a.model:
        config['model_path'] = a.model
    if a.port:
        config['port'] = a.port
    if a.browser:
        config['browser'] = a.browser
    if a.no_ros:
        config['ros'] = {'enabled': False}
    # Best effort below navigation: the service and its camera worker (child processes inherit the
    # niceness) yield the CPU to the engine, Nav2 and the Gateway; the per-robot rate falls by itself.
    niceness = int(config.get('nice', 10))
    if niceness:
        os.nice(niceness)
    deadline = time.monotonic()+60
    while True:
        try:
            available = gateway_robots(config['gateway'])
            break
        except OSError:
            if time.monotonic() > deadline:
                sys.exit(f"Gateway {config['gateway']} unreachable: start the lite twin first")
            time.sleep(1)
    robots = available if config['robots'] in ('all', '', None) else [r.strip() for r in str(config['robots']).split(',')]
    missing = set(robots)-set(available)
    if missing:
        sys.exit(f'robots {sorted(missing)} are not in the world state ({available})')
    print(f'perception for {robots} on http://{config["host"]}:{config["port"]}', flush=True)
    uvicorn.run(create_app(config, robots), host=config['host'], port=config['port'], log_level='warning', ws_max_size=64*2**20)


if __name__ == '__main__':
    main()
