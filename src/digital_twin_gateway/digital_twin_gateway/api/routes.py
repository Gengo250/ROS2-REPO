"""Versioned endpoints: read-only telemetry, plus validated world commands (obstacles, missions)."""
from fastapi import APIRouter, Request, Response, WebSocket, HTTPException
from fastapi.responses import FileResponse
from ..scene.models import WarehouseScene
from ..scene.adapter import load_scene
from ..state.models import Scenario
from ..state.models import ConnectionStatus, TwinState, TwinStateV2
from ..state.models import NavigationStatus, RouteGraph, RouteStatus
from ..routes import graph_path, load_route_graph
from ..world.models import CommandResult, MissionRequest, ObstacleRequest, WorldState

router = APIRouter()


@router.get('/api/v1/navigation', response_model=NavigationStatus)
async def navigation(request: Request):
    return request.app.state.manager.navigation_snapshot()


@router.get('/api/v1/routes/graph', response_model=RouteGraph)
def route_graph(request: Request):
    try:
        return load_route_graph(graph_path(request.app.state.manager.settings.route_graph_path))
    except (ValueError, OSError, KeyError) as exc:
        raise HTTPException(503, 'Route graph unavailable; build and install amr_route_planner') from exc


@router.get('/api/v1/routes/status', response_model=RouteStatus)
async def route_status(request: Request):
    return request.app.state.manager.route_snapshot()


@router.get('/api/v1/health')
async def health():
    return {'gateway': 'online', 'protocol_version': '1.0'}


@router.get('/api/v1/status', response_model=ConnectionStatus)
async def status(request: Request):
    snapshot = request.app.state.manager.snapshot()
    hubs = (request.app.state.hub, request.app.state.hub_v2, request.app.state.hub_world)
    return snapshot.connection.model_copy(update={'active_websocket_clients': sum(len(h.clients) for h in hubs)})


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


# ---------------------------------------------------------------- world-state v1 (LIVE frontend)
@router.get('/api/v1/world', response_model=WorldState)
async def world(request: Request):
    return request.app.state.world_latest


@router.websocket('/ws/v1/world')
async def world_socket(socket: WebSocket):
    await socket.app.state.hub_world.serve(socket)


@router.get('/api/v1/world/scene')
def world_scene(request: Request):
    """The compiled campus scene, byte-identical to what the engine loaded (hash inside)."""
    path = request.app.state.scene_path
    if path is None or not path.is_file():
        raise HTTPException(503, 'Campus scene not installed; build atlas_scene')
    return FileResponse(path, media_type='application/json')


def commands(request: Request):
    runtime = getattr(request.app.state, 'runtime', None)
    command = getattr(runtime, 'commands', None)
    if command is None:
        raise HTTPException(503, 'Commands unavailable: ROS or the lite engine interfaces are not running')
    return command


def result(response: Response, value: CommandResult, created=False):
    # A rejection is a valid answer from the backend, not a transport error.
    response.status_code = (201 if created else 200) if value.accepted else 409
    return value


@router.post('/api/v1/obstacles', response_model=CommandResult)
async def add_obstacle(body: ObstacleRequest, request: Request, response: Response):
    return result(response, await commands(request).add_obstacle(body), created=True)


@router.delete('/api/v1/obstacles/{identifier}', response_model=CommandResult)
async def remove_obstacle(identifier: str, request: Request, response: Response):
    if not identifier or len(identifier) > 64:
        raise HTTPException(422, 'invalid obstacle id')
    return result(response, await commands(request).remove_obstacle(identifier))


@router.delete('/api/v1/obstacles', response_model=CommandResult)
async def clear_obstacles(request: Request, response: Response):
    return result(response, await commands(request).remove_obstacle(''))


@router.post('/api/v1/missions', response_model=CommandResult)
async def start_mission(body: MissionRequest, request: Request, response: Response):
    return result(response, await commands(request).start_mission(body), created=True)


@router.delete('/api/v1/missions/{robot_id}', response_model=CommandResult)
async def cancel_mission(robot_id: str, request: Request, response: Response):
    return result(response, await commands(request).cancel_mission(robot_id))
