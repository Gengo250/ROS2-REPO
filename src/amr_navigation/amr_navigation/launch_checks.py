"""Fail before starting conflicting localization systems in one ROS domain."""
import time
import rclpy

def exclusive_mode():
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('navigation_launch_check', context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        deadline = time.monotonic()+3.
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.1)
        conflicts = set(node.get_node_names()) & {'slam_toolbox','amcl','map_server','bt_navigator','controller_server'}
        if conflicts:
            raise RuntimeError('Stop the existing mapping/localization/navigation mode first: '+', '.join(sorted(conflicts)))
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
