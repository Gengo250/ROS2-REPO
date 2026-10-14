"""Read-only versioned endpoints."""
from fastapi import APIRouter, Request, WebSocket
from ..state.models import ConnectionStatus, TwinState

router = APIRouter()


@router.get('/api/v1/health')
async def health():
    return {'gateway': 'online', 'protocol_version': '1.0'}


@router.get('/api/v1/status', response_model=ConnectionStatus)
async def status(request: Request):
    snapshot = request.app.state.manager.snapshot()
    return snapshot.connection.model_copy(update={
        'active_websocket_clients': len(request.app.state.hub.clients)})


@router.get('/api/v1/state', response_model=TwinState)
async def state(request: Request):
    return request.app.state.manager.snapshot()


@router.websocket('/ws/v1/twin')
async def twin(socket: WebSocket):
    await socket.app.state.hub.serve(socket)
