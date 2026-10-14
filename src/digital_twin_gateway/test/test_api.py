import asyncio
import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect
from digital_twin_gateway.api.app import create_app
from digital_twin_gateway.api.websocket import BroadcastHub
from digital_twin_gateway.config import Settings
from conftest import Runtime
from test_state import assert_schema


def test_http_lifecycle_and_cors():
    app = create_app(Settings(), Runtime)
    with TestClient(app) as client:
        assert client.get('/api/v1/health').json()['gateway'] == 'online'
        status = client.get('/api/v1/status')
        assert status.status_code == 200
        assert_schema(status.json(), 'connection-status')
        data = client.get('/api/v1/state').json()
        assert_schema(data, 'twin-state')
        assert data['robots'][0]['pose'] is None
        assert data['world']['status'] == 'unknown'
        assert client.post('/api/v1/state').status_code == 405
        for origin, allowed in [('http://localhost:5173', True), ('https://untrusted.test', False)]:
            r = client.get('/api/v1/health', headers={'Origin': origin})
            assert ('access-control-allow-origin' in r.headers) == allowed
    assert app.state.runtime.stopped
    assert not app.state.manager.ros_initialized
    assert app.state.hub.closed


def test_websocket_initial_updates_multiple_clients_and_disconnect():
    app = create_app(Settings(publication_hz=50), Runtime)
    with TestClient(app) as client:
        with client.websocket_connect('/ws/v1/twin') as a:
            first = a.receive_json()
            assert_schema(first, 'twin-state')
            with client.websocket_connect('/ws/v1/twin?protocol_version=1.0') as b:
                assert b.receive_json()['session_id'] == first['session_id']
                assert client.get('/api/v1/status').json()['active_websocket_clients'] == 2
                assert a.receive_json()['sequence'] > first['sequence']
                assert b.receive_json()['sequence'] > first['sequence']
            assert client.get('/api/v1/status').json()['active_websocket_clients'] == 1
        assert client.get('/api/v1/status').json()['active_websocket_clients'] == 0
        with client.websocket_connect('/ws/v1/twin') as again:
            assert again.receive_json()['sequence'] >= first['sequence']


@pytest.mark.parametrize('url,origin', [('/ws/v1/twin?protocol_version=2', None),
                                      ('/ws/v1/twin', 'https://evil.test')])
def test_ws_rejects_origin_and_protocol(url, origin):
    with TestClient(create_app(Settings(), Runtime)) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect(url, headers={'Origin': origin} if origin else {}):
                pass


def test_ws_read_only_and_client_limit():
    with TestClient(create_app(Settings(max_clients=1), Runtime)) as client:
        with client.websocket_connect('/ws/v1/twin') as socket:
            socket.receive_json()
            with pytest.raises(WebSocketDisconnect):
                with client.websocket_connect('/ws/v1/twin'):
                    pass
            socket.send_text('move robot')
            with pytest.raises(WebSocketDisconnect):
                while True:
                    socket.receive_json()


async def test_latest_queue_is_bounded():
    hub = BroadcastHub(Settings())
    queue = asyncio.Queue(maxsize=1)
    hub.clients[object()] = queue
    for index in range(10000):
        hub.publish(str(index))
    assert queue.qsize() == 1 and queue.get_nowait() == '9999'


async def test_slow_send_times_out_without_blocking_other_client():
    class Socket:
        headers = {}
        query_params = {}
        def __init__(self, slow):
            self.slow, self.sent, self.closed = slow, [], False
        async def accept(self):
            pass
        async def send_text(self, value):
            if self.slow:
                await asyncio.Event().wait()
            self.sent.append(value)
        async def receive(self):
            await asyncio.Event().wait()
        async def close(self, **kwargs):
            self.closed = True
    hub = BroadcastHub(Settings(send_timeout=.03))
    hub.publish('initial')
    slow, fast = Socket(True), Socket(False)
    tasks = [asyncio.create_task(hub.serve(s)) for s in (slow, fast)]
    await asyncio.sleep(.01)
    hub.publish('newest')
    await asyncio.sleep(.05)
    assert slow.closed and slow not in hub.clients
    assert fast.sent == ['initial', 'newest']
    await hub.close()
    assert fast.closed
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


def test_startup_failure_releases_runtime():
    instances = []
    class FailingRuntime(Runtime):
        def __init__(self, manager, settings):
            super().__init__(manager, settings)
            instances.append(self)
        def start(self):
            raise RuntimeError('initialization failed')
    with pytest.raises(RuntimeError, match='initialization failed'):
        with TestClient(create_app(Settings(), FailingRuntime)):
            pass
    assert instances[0].stopped


async def test_peer_disappearing_during_close_is_normal():
    class GoneSocket:
        headers = {}
        query_params = {}
        async def accept(self):
            pass
        async def receive(self):
            return {'type': 'websocket.disconnect', 'code': 1006}
        async def send_text(self, value):
            raise WebSocketDisconnect(code=1006)
        async def close(self, **kwargs):
            raise WebSocketDisconnect(code=1006)
    hub = BroadcastHub(Settings())
    hub.publish('initial')
    await hub.serve(GoneSocket())
    assert not hub.clients
    hub.clients[GoneSocket()] = asyncio.Queue(maxsize=1)
    await hub.close()


def test_ws_binary_messages_are_rejected():
    with TestClient(create_app(Settings(), Runtime)) as client:
        with client.websocket_connect('/ws/v1/twin') as socket:
            socket.receive_json()
            socket.send_bytes(b'command')
            with pytest.raises(WebSocketDisconnect):
                while True:
                    socket.receive_json()
