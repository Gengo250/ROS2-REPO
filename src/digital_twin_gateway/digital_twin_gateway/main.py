"""Run exactly one API worker; each process owns one ROS context."""
import uvicorn
from .api.app import create_app
from .config import Settings


class GatewayServer(uvicorn.Server):
    def handle_exit(self, sig, frame):
        # Terminal SIGINT and ROS launch forwarding can arrive together. Uvicorn's
        # default second-SIGINT path skips lifespan cleanup, leaking the ROS thread.
        # Let the bounded graceful shutdown finish; launch retains its TERM/KILL
        # escalation if the process cannot finish.
        if not self.should_exit:
            super().handle_exit(sig, frame)


def main():
    settings = Settings.from_env()
    server = GatewayServer(uvicorn.Config(
        create_app(settings), host=settings.host, port=settings.port,
        workers=1, ws_max_size=4096, ws_max_queue=1, timeout_graceful_shutdown=5))
    try:
        server.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
