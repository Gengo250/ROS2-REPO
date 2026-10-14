"""FastAPI lifespan owns ROS, snapshot publication and client cleanup."""
import asyncio
from contextlib import asynccontextmanager, suppress
from time import monotonic
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ..config import Settings
from ..state.state_manager import StateManager
from .routes import router
from .websocket import BroadcastHub


def create_app(settings=None, runtime_factory=None, manager=None):
    settings = settings or Settings.from_env()
    manager = manager or StateManager(settings)
    if runtime_factory is None:
        from ..ros.node import RosRuntime
        runtime_factory = RosRuntime
    hub = BroadcastHub(settings)

    @asynccontextmanager
    async def lifespan(app):
        runtime = runtime_factory(manager, settings)
        publisher = None
        try:
            runtime.start()
            hub.publish(manager.publish().model_dump_json())

            async def publish():
                last = monotonic()
                period = 1 / settings.publication_hz
                while True:
                    await asyncio.sleep(period)
                    now = monotonic()
                    snapshot = manager.publish(len(hub.clients), 1 / (now-last))
                    hub.publish(snapshot.model_dump_json())
                    last = now

            publisher = asyncio.create_task(publish(), name='twin-publisher')
            app.state.runtime = runtime
            yield
        finally:
            if publisher:
                publisher.cancel()
                with suppress(asyncio.CancelledError):
                    await publisher
            await hub.close()
            runtime.stop()

    app = FastAPI(title='Digital Twin Gateway', version='1.0', lifespan=lifespan)
    app.state.manager, app.state.hub = manager, hub
    app.add_middleware(CORSMiddleware, allow_origins=list(settings.allowed_origins),
                       allow_methods=['GET'], allow_headers=['Accept'], allow_credentials=False)
    app.include_router(router)
    return app
