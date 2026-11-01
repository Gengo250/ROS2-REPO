#!/usr/bin/env python3
"""Exercise the Nav2 Route Server on the campus graph and check its routes against Dijkstra.

Starts nav2_route's route_server with the GeoJSON graph (map frame), activates it, requests
ComputeRoute by node id for station pairs, and compares each route's cost and node sequence with
an independent Dijkstra over the same file costs. Pure graph planning: no TF or costmap needed.
Usage: evaluate_route_server.py --graph campus_graph.geojson --output report.json
"""
import argparse
import heapq
import itertools
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeRoute


def dijkstra(edges, start, goal):
    best, queue, previous = {start: 0.}, [(0., start)], {}
    while queue:
        cost, node = heapq.heappop(queue)
        if node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return cost, path[::-1]
        if cost > best.get(node, float('inf')):
            continue
        for target, c in edges.get(node, ()):
            if cost+c < best.get(target, float('inf')):
                best[target], previous[target] = cost+c, node
                heapq.heappush(queue, (cost+c, target))
    return None, []


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--graph', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    graph = json.loads(a.graph.read_text())
    names = {f['properties']['id']: f['properties'].get('name') for f in graph['features'] if f['geometry']['type'] == 'Point'}
    edges = {}
    for f in graph['features']:
        if f['geometry']['type'] != 'Point':
            pr = f['properties']
            edges.setdefault(pr['startid'], []).append((pr['endid'], pr['cost']))
    stations = sorted((i, n) for i, n in names.items() if n and not n.startswith('n'))
    params = Path(tempfile.mkdtemp(prefix='route_server_'))/'params.yaml'
    params.write_text(json.dumps({'route_server': {'ros__parameters': {
        'graph_filepath': str(a.graph), 'route_frame': 'map', 'base_frame': 'base_link', 'max_planning_time': 2.0,
        'edge_cost_functions': ['DistanceScorer'], 'DistanceScorer': {'plugin': 'nav2_route::DistanceScorer'},
        'operations': ['ReroutingService'], 'ReroutingService': {'plugin': 'nav2_route::ReroutingService'}}}}))
    server = subprocess.Popen(['ros2', 'run', 'nav2_route', 'route_server', '--ros-args', '--params-file', str(params)],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    report = {'graph': str(a.graph), 'stations': len(stations), 'pairs': []}
    try:
        time.sleep(3)
        for transition in ('configure', 'activate'):
            out = subprocess.run(['ros2', 'lifecycle', 'set', '/route_server', transition], capture_output=True, text=True, timeout=30)
            report[f'lifecycle_{transition}'] = out.stdout.strip() or out.stderr.strip()
        rclpy.init()
        node = rclpy.create_node('route_server_evaluation')
        client = ActionClient(node, ComputeRoute, 'compute_route')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('compute_route action unavailable')
        for (si, sn), (gi, gn) in itertools.permutations(stations, 2):
            goal = ComputeRoute.Goal(start_id=si, goal_id=gi, use_start=False, use_poses=False)
            started = time.monotonic()
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            handle = future.result()
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=10)
            result = result_future.result().result
            wall = time.monotonic()-started
            expected_cost, expected_path = dijkstra(edges, si, gi)
            route_nodes = [n.nodeid for n in result.route.nodes]
            report['pairs'].append({'from': sn, 'to': gn, 'error_code': result.error_code,
                                    'route_cost': result.route.route_cost, 'dijkstra_cost': expected_cost,
                                    'same_nodes': route_nodes == expected_path, 'nodes': len(route_nodes),
                                    'planning_time_ms': result.planning_time.sec*1e3+result.planning_time.nanosec/1e6,
                                    'request_wall_ms': wall*1e3, 'path_points': len(result.path.poses)})
        node.destroy_node()
        rclpy.shutdown()
    finally:
        os.killpg(server.pid, 2)
        try:
            server.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, 9)
    pairs = report['pairs']
    ok = [x for x in pairs if x['error_code'] == 0]
    report['summary'] = {
        'requests': len(pairs), 'succeeded': len(ok),
        'cost_matches_dijkstra': sum(1 for x in ok if x['dijkstra_cost'] is not None and abs(x['route_cost']-x['dijkstra_cost']) < 1e-3*max(1, x['dijkstra_cost'])),
        'same_node_sequence': sum(1 for x in ok if x['same_nodes']),
        'max_planning_time_ms': max((x['planning_time_ms'] for x in ok), default=None),
        'median_planning_time_ms': sorted(x['planning_time_ms'] for x in ok)[len(ok)//2] if ok else None}
    a.output.write_text(json.dumps(report, indent=1)+'\n')
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
