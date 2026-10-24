"""Read-only versioned endpoints."""
from fastapi import APIRouter, Request, WebSocket, HTTPException
from ..scene.models import WarehouseScene
from ..scene.adapter import load_scene
from ..state.models import Scenario
from ..state.models import ConnectionStatus, TwinState, TwinStateV2
from ..state.models import NavigationStatus

router = APIRouter()


@router.get('/api/v1/navigation', response_model=NavigationStatus)
async def navigation(request: Request):
    return request.app.state.manager.navigation_snapshot()


@router.get('/api/v1/health')
async def health():
    return {'gateway': 'online', 'protocol_version': '1.0'}


@router.get('/api/v1/status', response_model=ConnectionStatus)
async def status(request: Request):
    snapshot = request.app.state.manager.snapshot()
    return snapshot.connection.model_copy(update={
        'active_websocket_clients': len(request.app.state.hub.clients)+len(request.app.state.hub_v2.clients)})


@router.get('/api/v1/state', response_model=TwinState)
async def state(request: Request):
    return request.app.state.manager.snapshot()


@router.websocket('/ws/v1/twin')
async def twin(socket: WebSocket):
    await socket.app.state.hub.serve(socket)


@router.get('/api/v1/scene', response_model=WarehouseScene)
def scene(request: Request, scenario: Scenario | None = None):
    configured = request.app.state.manager.snapshot().world.scenario
    try:
        return load_scene(scenario or configured or 'normal', configured)
    except (ValueError, OSError, KeyError) as exc:
        raise HTTPException(503, 'Scene sources invalid; regenerate warehouse assets') from exc


@router.get('/api/v2/state', response_model=TwinStateV2)
async def state_v2(request: Request):
    return request.app.state.manager.snapshot(2)


@router.websocket('/ws/v2/twin')
async def twin_v2(socket: WebSocket):
    await socket.app.state.hub_v2.serve(socket)
