from digital_twin_gateway.config import Settings
from digital_twin_gateway.ros.node import RosRuntime
from digital_twin_gateway.state.state_manager import StateManager


def test_real_ros_start_subscriptions_and_graceful_stop():
    settings = Settings()
    state = StateManager(settings)
    runtime = RosRuntime(state, settings)
    try:
        runtime.start()
        assert state.ros_initialized and runtime.thread.is_alive()
        topics = dict(runtime.node.get_topic_names_and_types())
        for topic in ('/clock', '/odom', '/tf', '/tf_static', '/joint_states'):
            assert topic in topics
        assert runtime.node.count_publishers('/cmd_vel') == 0
    finally:
        runtime.stop()
    assert not runtime.thread.is_alive()
    assert not state.ros_initialized
    assert not runtime.context.ok()
    runtime.stop()  # cleanup is idempotent


def test_duplicate_launch_signals_keep_graceful_shutdown():
    import signal
    import uvicorn
    from digital_twin_gateway.main import GatewayServer
    server = GatewayServer(uvicorn.Config('unused:test'))
    server.handle_exit(signal.SIGINT, None)
    server.handle_exit(signal.SIGINT, None)
    assert server.should_exit
    assert not server.force_exit
