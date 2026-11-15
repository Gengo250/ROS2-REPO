"""Channel to the onboard-camera render worker (camera.html in a Chromium process).

``RenderChannel`` is the asyncio side (one worker WebSocket, request/response by job id, one job in
flight at a time: the worker renders sequentially). ``serve_app`` mounts it and the built web
bundle (web/digital-twin/dist) in a FastAPI app. ``launch_browser`` starts Chromium:
 * ``gpu``: a minimised Chromium window on the desktop session, rendering on the display GPU
   (here Intel UHD 630 through Mesa/ANGLE);
 * ``headless``: no window; Chromium then renders with SwiftShader (CPU), much slower.
 * ``nvidia``: no window; Chromium's ANGLE renders through EGL on the NVIDIA driver (glvnd vendor
   file), so a hybrid laptop renders camera frames on the discrete GPU and leaves the display GPU to
   the twin's page (GTX 1650: about 19 ms per RGB + ID frame; same pixels as the Intel renderer to
   0.04 grey levels on average, identical instance-ID passes — see the Phase 7 document).
``auto`` picks ``nvidia`` when that driver is present, else ``gpu`` on a desktop session, else ``headless``.
``RenderWorker`` wraps all of it for synchronous scripts (dataset generation, benchmarks).
No image is ever produced without the renderer: a missing or failed worker raises.
"""
import asyncio
import itertools
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlencode
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from .frames import Frame, parse
from .paths import WEB_DIST, chromium


class RenderError(RuntimeError):
    pass


class RenderChannel:
    def __init__(self):
        self.socket: WebSocket | None = None
        self.hello: dict | None = None
        self.ready = asyncio.Event()
        self.pending: dict[int, asyncio.Future] = {}
        self.ids = itertools.count(1)
        self.lock = asyncio.Lock()
        self.disconnect_reason = None

    async def serve(self, socket: WebSocket):
        await socket.accept()
        if self.socket is not None:
            await socket.close(code=4409, reason='a render worker is already connected')
            return
        self.socket = socket
        try:
            while True:
                message = await socket.receive()
                if message.get('type') == 'websocket.disconnect':
                    break
                if message.get('bytes') is not None:
                    frame = parse(message['bytes'])
                    future = self.pending.pop(frame.header['job_id'], None)
                    if future and not future.done():
                        future.set_result(frame)
                elif message.get('text') is not None:
                    data = json.loads(message['text'])
                    if data.get('type') == 'hello':
                        self.hello = data
                        self.ready.set()
                    elif data.get('type') == 'error':
                        future = self.pending.pop(data.get('job_id'), None)
                        if future and not future.done():
                            future.set_exception(RenderError(data.get('reason', 'render failed')))
        except WebSocketDisconnect as error:
            self.disconnect_reason = f'worker closed the channel (code {error.code})'
        except Exception as error:   # malformed frame: drop the worker rather than guess
            self.disconnect_reason = f'channel error: {error!r}'
            raise
        finally:
            self.socket, self.hello = None, None
            self.ready.clear()
            for future in self.pending.values():
                if not future.done():
                    future.set_exception(RenderError('render worker disconnected'))
            self.pending.clear()

    @property
    def connected(self):
        return self.socket is not None and self.hello is not None

    async def request(self, job: dict, timeout=30.) -> Frame:
        if not self.connected:
            raise RenderError(f'no render worker connected ({self.disconnect_reason or "never connected"})')
        async with self.lock:
            job_id = next(self.ids)
            future = asyncio.get_running_loop().create_future()
            self.pending[job_id] = future
            await self.socket.send_text(json.dumps({**job, 'job_id': job_id}))
            try:
                return await asyncio.wait_for(future, timeout)
            finally:
                self.pending.pop(job_id, None)


def mount_worker(app: FastAPI, channel: RenderChannel, dist=WEB_DIST):
    """WebSocket /ws/render for the worker and the built web bundle (camera.html, scene, robots)."""
    if not (dist/'camera.html').is_file():
        raise FileNotFoundError(f'{dist}/camera.html missing: build the web bundle (cd web/digital-twin && npm run build)')

    @app.websocket('/ws/render')
    async def render_socket(socket: WebSocket):
        await channel.serve(socket)
    app.mount('/', StaticFiles(directory=str(dist), html=True), name='web')


NVIDIA_EGL_VENDOR = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
BROWSER_MODES = ('auto', 'nvidia', 'gpu', 'headless')


def nvidia_available():
    return os.path.isfile(NVIDIA_EGL_VENDOR) and os.path.exists('/dev/nvidia0')


def browser_mode(mode='auto'):
    if mode == 'auto':
        if nvidia_available():
            return 'nvidia'
        return 'gpu' if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') else 'headless'
    if mode not in BROWSER_MODES:
        raise ValueError(f'browser mode must be one of {BROWSER_MODES}')
    if mode == 'nvidia' and not nvidia_available():
        raise RuntimeError(f'browser mode nvidia: no NVIDIA EGL driver ({NVIDIA_EGL_VENDOR}, /dev/nvidia0)')
    return mode


def launch_browser(url, mode='auto', log_path=None):
    """Start Chromium on the worker page; returns (process, profile directory)."""
    mode = browser_mode(mode)
    env = dict(os.environ)
    profile = tempfile.mkdtemp(prefix='atlas-camera-worker-')
    # --no-sandbox: Ubuntu 24.04 restricts the unprivileged user namespaces Chromium's sandbox needs
    # (AppArmor); Playwright runs the project's browser tests the same way. The worker only loads
    # this repository's own bundle from 127.0.0.1.
    flags = ['--no-sandbox', '--no-first-run', '--no-default-browser-check', f'--user-data-dir={profile}', '--disable-extensions',
             '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
             '--disable-backgrounding-occluded-windows', '--ignore-gpu-blocklist', '--disable-features=Translate',
             '--password-store=basic']
    if mode == 'gpu':
        # XWayland window, minimised: rendering happens in off-screen render targets on the GPU.
        flags += ['--ozone-platform=x11', '--start-minimized', '--window-size=320,200', f'--app={url}']
    elif mode == 'nvidia':
        flags += ['--headless=new', '--use-angle=gl-egl', url]
        env['__EGL_VENDOR_LIBRARY_FILENAMES'] = NVIDIA_EGL_VENDOR
    else:
        flags += ['--headless=new', url]
    log = open(log_path, 'ab') if log_path else subprocess.DEVNULL
    process = subprocess.Popen([chromium(), *flags], stdout=log, stderr=log, stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    return process, profile


class RenderWorker:
    """Synchronous facade: local server + Chromium worker; ``render(job)`` returns a Frame."""

    def __init__(self, port=8765, mode='auto', gateway=None, log_path=None, timeout=90.):
        import uvicorn
        self.channel = RenderChannel()
        app = FastAPI()
        mount_worker(app, self.channel)
        self.loop = asyncio.new_event_loop()
        config = uvicorn.Config(app, host='127.0.0.1', port=port, log_level='warning', loop='asyncio', ws_max_size=64*2**20)
        self.server = uvicorn.Server(config)
        self.thread = threading.Thread(target=self._run, name='render-worker-server', daemon=True)
        self.thread.start()
        deadline = time.monotonic()+15
        while not self.server.started:
            if time.monotonic() > deadline:
                raise RenderError(f'render server did not start on port {port}')
            time.sleep(.05)
        query = {'channel': f'ws://127.0.0.1:{port}/ws/render'}
        if gateway:
            query['gateway'] = gateway
        self.mode = browser_mode(mode)
        self.process, self.profile = launch_browser(f'http://127.0.0.1:{port}/camera.html?{urlencode(query)}', self.mode, log_path)
        try:
            asyncio.run_coroutine_threadsafe(asyncio.wait_for(self.channel.ready.wait(), timeout), self.loop).result(timeout+5)
        except Exception as error:
            self.close()
            raise RenderError(f'render worker did not connect within {timeout:.0f} s ({self.mode})') from error
        self.hello = self.channel.hello

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.server.serve())

    def render(self, job, timeout=60.) -> Frame:
        return asyncio.run_coroutine_threadsafe(self.channel.request(job, timeout), self.loop).result(timeout+5)

    def close(self):
        if getattr(self, 'process', None) and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, 15)
                self.process.wait(10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.process.pid, 9)
                except ProcessLookupError:
                    pass
        if getattr(self, 'profile', None):
            shutil.rmtree(self.profile, ignore_errors=True)
        self.server.should_exit = True
        self.thread.join(timeout=10)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
