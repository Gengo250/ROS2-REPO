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
    def __init__(self, manager, settings):
        self.manager, self.settings = manager, settings
        self.context = Context()
        self.node = self.executor = self.thread = None

    def start(self):
        try:
            # Uvicorn owns process signals; ROS does not install a second handler.
            rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
            self.node = Node('digital_twin_gateway', context=self.context)
            self.subscriptions = subscribe(self.node, self.manager, self.settings)
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
