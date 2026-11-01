"""FastAPI lifespan owns ROS, snapshot publication and client cleanup."""
import asyncio
import json
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from time import monotonic
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.gzip import GZipMiddleware
from ..config import Settings
from ..state.state_manager import StateManager
from ..world import registration as world_registration
from ..world.store import WorldStore
from .routes import router
from .websocket import BroadcastHub


def default_path(package, relative):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return Path(get_package_share_directory(package))/relative
    except PackageNotFoundError:
        return None


def scene_stations(path):
    """Station goal poses (world frame) from the campus scene file, keyed by id."""
    if path is None or not Path(path).is_file():
        return {}
    return {s['id']: s for s in json.loads(Path(path).read_text()).get('stations', ())}


def create_app(settings=None, runtime_factory=None, manager=None, world=None):
    settings = settings or Settings.from_env()
    manager = manager or StateManager(settings)
    scene_path = Path(settings.world_scene_path) if settings.world_scene_path else \
        default_path('atlas_scene', 'config/atlas_campus.scene.json')
    if world is None:
        registration_path = settings.world_registration_path or \
            default_path('amr_navigation', 'maps/campus_registration.json')
        world = WorldStore(settings, world_registration.load(registration_path))
    world.stations = scene_stations(scene_path)
    if runtime_factory is None:
        from ..ros.node import RosRuntime
        runtime_factory = RosRuntime
    hub = BroadcastHub(settings)
    hub_v2 = BroadcastHub(settings, '2.0')
    hub_world = BroadcastHub(settings, '1.0')

    @asynccontextmanager
    async def lifespan(app):
        runtime = runtime_factory(manager, settings, world)
        tasks = []
        try:
            runtime.start()
            hub.publish(manager.publish().model_dump_json())
            hub_v2.publish(manager.snapshot(2).model_dump_json())
            app.state.world_latest = world.snapshot()
            hub_world.publish(app.state.world_latest.model_dump_json())

            async def publish():
                last = monotonic()
                period = 1 / settings.publication_hz
                while True:
                    await asyncio.sleep(period)
                    now = monotonic()
                    clients = len(hub.clients)+len(hub_v2.clients)+len(hub_world.clients)
                    snapshot = manager.publish(clients, 1 / (now-last))
                    hub.publish(snapshot.model_dump_json())
                    hub_v2.publish(manager.snapshot(2).model_dump_json())
                    last = now

            async def publish_world():
                # One snapshot and one serialization per tick, shared by every client.
                period = 1 / settings.world_publication_hz
                deadline = monotonic()
                while True:
                    deadline += period
                    await asyncio.sleep(max(0., deadline-monotonic()))
                    if monotonic()-deadline > period:
                        deadline = monotonic()
                    app.state.world_latest = world.snapshot()
                    hub_world.publish(app.state.world_latest.model_dump_json())

            tasks = [asyncio.create_task(publish(), name='twin-publisher'),
                     asyncio.create_task(publish_world(), name='world-publisher')]
            app.state.runtime = runtime
            yield
        finally:
            for task in tasks:
                task.cancel()
                with suppress(asyncio.CancelledError):
                    await task
            await hub.close()
            await hub_v2.close()
            await hub_world.close()
            runtime.stop()

    app = FastAPI(title='Digital Twin Gateway', version='1.0', lifespan=lifespan)
    app.state.manager, app.state.hub = manager, hub
    app.state.hub_v2, app.state.hub_world, app.state.world = hub_v2, hub_world, world
    app.state.scene_path = scene_path
    app.add_middleware(GZipMiddleware, minimum_size=4096)
    app.add_middleware(CORSMiddleware, allow_origins=list(settings.allowed_origins),
                       allow_methods=['GET', 'POST', 'DELETE'], allow_headers=['Accept', 'Content-Type'],
                       allow_credentials=False)
    app.include_router(router)
    return app
