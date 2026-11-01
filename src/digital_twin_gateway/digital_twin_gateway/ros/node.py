"""One owned ROS context and one managed executor thread per gateway process."""
import logging
from threading import Thread
import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from .subscriptions import subscribe


class RosRuntime:
    def __init__(self, manager, settings, world=None):
        self.manager, self.settings, self.world = manager, settings, world
        self.context = Context()
        self.node = self.executor = self.thread = None
        self.commands = None

    def start(self):
        try:
            # Uvicorn owns process signals; ROS does not install a second handler.
            rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
            self.node = Node('digital_twin_gateway', context=self.context)
            self.subscriptions = subscribe(self.node, self.manager, self.settings)
            if self.world is not None:
                from .world import Commands, subscribe_world
                self.subscriptions += subscribe_world(self.node, self.world, self.settings)
                try:
                    self.commands = Commands(self.node, self.world, self.settings, self.world.stations)
                except ImportError:
                    logging.warning('Lite interfaces or Nav2 messages missing: commands disabled')
                self.world.set_ros_initialized(True)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.manager.set_ros_initialized(True)
            self.thread = Thread(target=self._spin, name='twin-ros-executor')
            self.thread.start()
        except BaseException:
            self.stop()
            raise

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception:
            logging.exception('ROS executor failed')
        finally:
            self.manager.set_ros_initialized(False)
            if self.world is not None:
                self.world.set_ros_initialized(False)

    def stop(self):
        if self.executor:
            self.executor.shutdown(timeout_sec=5)
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError('ROS executor did not stop')
        if self.node:
            self.node.destroy_node()
            self.node = None
        if self.context.ok():
            self.context.try_shutdown()
        self.manager.set_ros_initialized(False)
        if self.world is not None:
            self.world.set_ros_initialized(False)
