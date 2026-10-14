"""Latest snapshot fan-out with bounded queues and independently timed sends."""
import asyncio
from fastapi import WebSocket, WebSocketDisconnect


class BroadcastHub:
    def __init__(self, settings):
        self.settings = settings
        self.clients = {}
        self.latest = ''
        self.closed = False

    def publish(self, payload):
        self.latest = payload
        for queue in self.clients.values():
            if queue.full():
                queue.get_nowait()
            queue.put_nowait(payload)

    async def serve(self, socket: WebSocket):
        origin = socket.headers.get('origin')
        if origin is not None and origin not in self.settings.allowed_origins:
            await socket.close(code=1008)
            return
        # Non-browser diagnostic clients may omit Origin. This is not authentication.
        if socket.query_params.get('protocol_version', '1.0') != '1.0':
            await socket.close(code=1002)
            return
        if self.closed or len(self.clients) >= self.settings.max_clients:
            await socket.close(code=1013)
            return
        # Reserve before the first await, so simultaneous handshakes respect the cap.
        queue = asyncio.Queue(maxsize=1)
        self.clients[socket] = queue
        tasks = []
        try:
            await socket.accept()
            if queue.empty():
                queue.put_nowait(self.latest)

            async def send():
                while True:
                    payload = await queue.get()
                    await asyncio.wait_for(socket.send_text(payload), self.settings.send_timeout)

            async def receive():
                # A read-only endpoint accepts no application commands or messages.
                message = await socket.receive()
                if message['type'] != 'websocket.disconnect':
                    await socket.close(code=1008, reason='Read-only telemetry endpoint')

            tasks = [asyncio.create_task(send()), asyncio.create_task(receive())]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, OSError, asyncio.TimeoutError):
            pass
        finally:
            self.clients.pop(socket, None)
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            try:
                await asyncio.wait_for(socket.close(code=1001), self.settings.send_timeout)
            except (WebSocketDisconnect, RuntimeError, OSError, asyncio.TimeoutError):
                pass

    async def close(self):
        self.closed = True
        async def close_one(socket):
            try:
                await asyncio.wait_for(socket.close(code=1001), self.settings.send_timeout)
            except (WebSocketDisconnect, RuntimeError, OSError, asyncio.TimeoutError):
                pass
        await asyncio.gather(*(close_one(s) for s in list(self.clients)))
