#!/usr/bin/env python3
"""Measured graph-construction and A*/Dijkstra planning metrics on identical inputs.

Every ordered pair of graph locations is planned with both algorithms on the same
graph and restrictions; costs must agree. Times are wall-clock medians of repeats.
"""
import argparse
import itertools
import json
from pathlib import Path
import statistics
import time
from amr_route_planner import graph_builder as gb
from amr_route_planner.cost_model import CostModel, Restrictions, exclusions
from amr_route_planner.graph_model import Graph
from amr_route_planner.route_planner import search


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source-tree', action='store_true')
    p.add_argument('--repeats', type=int, default=50)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    paths = gb.Paths.source_tree() if a.source_tree else gb.Paths()
    builds = []
    for _ in range(5):
        began = time.perf_counter()
        artifact = gb.build_artifact(paths)
        builds.append(time.perf_counter()-began)
    settings = gb.load_settings(paths.file('planner_config'))
    model = CostModel.from_settings(settings)
    graph = Graph.from_dict(artifact)
    report = {'graph_version': artifact['graph_version'], 'graph_build_s': {'median': statistics.median(builds), 'runs': builds},
              'nodes': len(graph.nodes), 'directed_edges': len(graph.edges), 'repeats': a.repeats, 'sets': {}}
    for name, restrictions in (('validated', Restrictions()), ('topology', Restrictions(require_validated=False))):
        edges, nodes = exclusions(artifact, restrictions)
        usable = sorted(n for n in graph.nodes if n not in nodes)
        rows = []
        for start, goal in itertools.permutations(usable, 2):
            row = {'start': start, 'goal': goal}
            for algorithm in ('astar', 'dijkstra'):
                times, result = [], None
                for _ in range(a.repeats):
                    result = search(graph, start, goal, model, algorithm, edges, nodes,
                                    start_yaw=graph.nodes[start].yaw, goal_yaw=graph.nodes[goal].yaw)
                    times.append(result.duration_s)
                row[algorithm] = {'found': result.found, 'cost': result.cost if result.found else None,
                                  'distance_m': result.distance, 'expanded_states': result.expanded_states,
                                  'visited_nodes': result.visited_nodes, 'median_s': statistics.median(times)}
            if row['astar']['found'] != row['dijkstra']['found'] or (
                    row['astar']['found'] and abs(row['astar']['cost']-row['dijkstra']['cost']) > 1e-6):
                raise SystemExit(f'Algorithms disagree for {start} → {goal}')
            rows.append(row)
        found = [r for r in rows if r['astar']['found']]
        summary = {'pairs': len(rows), 'routes_found': len(found)}
        for algorithm in ('astar', 'dijkstra'):
            summary[algorithm] = {
                'median_planning_s': statistics.median(r[algorithm]['median_s'] for r in rows) if rows else None,
                'max_planning_s': max((r[algorithm]['median_s'] for r in rows), default=None),
                'mean_expanded_states': statistics.mean(r[algorithm]['expanded_states'] for r in rows) if rows else None,
                'mean_visited_nodes': statistics.mean(r[algorithm]['visited_nodes'] for r in rows) if rows else None}
        if found:
            summary['astar_fewer_expansions_pairs'] = sum(r['astar']['expanded_states'] < r['dijkstra']['expanded_states'] for r in found)
            summary['equal_expansions_pairs'] = sum(r['astar']['expanded_states'] == r['dijkstra']['expanded_states'] for r in found)
        report['sets'][name] = {'summary': summary, 'pairs': rows}
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'graph_build_median_s': report['graph_build_s']['median'],
                      **{k: v['summary'] for k, v in report['sets'].items()}}, indent=2))


if __name__ == '__main__':
    main()
