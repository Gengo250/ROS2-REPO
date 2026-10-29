"""Graph model and search: hand-verifiable graphs and exhaustive cross-checks."""
import itertools
import math
import random
import pytest
from amr_route_planner.cost_model import CostModel, Restrictions, exclusions
from amr_route_planner.graph_model import Edge, Graph, Node, components, heading, validate_structure
from amr_route_planner.route_planner import RoutingError, search


def build(points, links, one_way=()):
    g = Graph()
    for name, (x, y) in points.items():
        g.add_node(Node(name, 'INTERSECTION', (x, y)))
    for a, b, *extra in links:
        d = math.dist(points[a], points[b])
        penalty = extra[0] if extra else 0.
        for s, t in ([(a, b)] if (a, b) in one_way else [(a, b), (b, a)]):
            g.add_edge(Edge(f'{s}->{t}', s, t, '--'.join(sorted((a, b))), 'corridor', 'c', d, d+penalty, 3.,
                            'one_way' if (a, b) in one_way else 'bidirectional'))
    return g


def lattice(n=4):
    """n×n unit grid; every monotone path between opposite corners has length 2(n-1)."""
    points = {f'{i}{j}': (i, j) for i in range(n) for j in range(n)}
    links = [(f'{i}{j}', f'{i+1}{j}') for i in range(n-1) for j in range(n)] + \
            [(f'{i}{j}', f'{i}{j+1}') for i in range(n) for j in range(n-1)]
    return build(points, links)


def brute_force(g, start, goal, model, start_yaw=None):
    """Exhaustive minimum over all simple directed paths (independent of the search code)."""
    best = math.inf
    out = g.adjacency()

    def walk(node, visited, cost, h):
        nonlocal best
        if node == goal:
            best = min(best, cost)
            return
        for e in out[node]:
            if e.target not in visited:
                walk(e.target, visited | {e.target}, cost+model.edge(e)+model.turn(h, heading(g, e)), heading(g, e))
    walk(start, {start}, 0., start_yaw)
    return best


@pytest.mark.parametrize('algorithm', ['astar', 'dijkstra'])
def test_lattice_shortest_distance_and_turn_tiebreak(algorithm):
    g = lattice()
    plain = search(g, '00', '33', CostModel(1., 0.), algorithm)
    assert plain.found and plain.distance == pytest.approx(6.) and plain.cost == pytest.approx(6.)
    # Twenty equal-length monotone paths; turn cost selects one with a single turn.
    turned = search(g, '00', '33', CostModel(1., 1.2857), algorithm)
    assert turned.distance == pytest.approx(6.)
    assert turned.cost == pytest.approx(6.+1.2857*math.pi/2)
    assert turned.nodes in (['00', '10', '20', '30', '31', '32', '33'], ['00', '01', '02', '03', '13', '23', '33'])


def test_astar_matches_dijkstra_and_brute_force_on_random_graphs():
    rng = random.Random(7)
    for trial in range(40):
        names = [f'n{i}' for i in range(7)]
        points = {n: (rng.uniform(0, 10), rng.uniform(0, 10)) for n in names}
        links = [(a, b, rng.choice([0., 0., 2.5])) for a, b in itertools.combinations(names, 2) if rng.random() < .45]
        g = build(points, links)
        model = CostModel(1., rng.choice([0., 1.2857]))
        start, goal = names[0], names[-1]
        expected = brute_force(g, start, goal, model, start_yaw=.3)
        a = search(g, start, goal, model, 'astar', start_yaw=.3)
        d = search(g, start, goal, model, 'dijkstra', start_yaw=.3)
        assert a.found == d.found == math.isfinite(expected)
        if a.found:
            assert a.cost == pytest.approx(expected) and d.cost == pytest.approx(expected)
            # Returned path really costs what is reported (no invalid shortcut).
            total, h = 0., .3
            for e in a.edges:
                total += model.edge(g.edges[e])+model.turn(h, heading(g, g.edges[e]))
                h = heading(g, g.edges[e])
            assert total == pytest.approx(a.cost)
            assert all(g.edges[e].source == s and g.edges[e].target == t for e, s, t in zip(a.edges, a.nodes, a.nodes[1:]))


def test_astar_expands_fewer_states_on_a_long_corridor_graph():
    points = {f'p{i}': (float(i), 0.) for i in range(30)} | {f'q{i}': (float(i), -3.) for i in range(30)}
    links = [(f'p{i}', f'p{i+1}') for i in range(29)] + [(f'q{i}', f'q{i+1}') for i in range(29)] + \
            [(f'p{i}', f'q{i}') for i in range(0, 30, 5)]
    g = build(points, links)
    a = search(g, 'p0', 'p29', CostModel(1., 1.2857), 'astar')
    d = search(g, 'p0', 'p29', CostModel(1., 1.2857), 'dijkstra')
    assert a.cost == pytest.approx(d.cost) == pytest.approx(29.)
    assert a.expanded_states < d.expanded_states


def test_blocked_disconnected_one_way_invalid_and_deterministic():
    g = lattice(3)
    model = CostModel(1., 1.)
    direct = search(g, '00', '20', model)
    assert direct.nodes == ['00', '10', '20']
    detour = search(g, '00', '20', model, excluded_edges={'00->10', '10->00'})
    assert detour.found and detour.distance == pytest.approx(4.) and '10' not in detour.nodes[:2]
    assert not search(g, '00', '20', model, excluded_nodes={'20'}).found
    cut = {e.id for e in g.edges.values() if '20' in (e.source, e.target)}
    assert not search(g, '00', '20', model, excluded_edges=cut).found
    with pytest.raises(RoutingError) as err:
        search(g, '00', 'missing', model)
    assert err.value.code == 'unknown_location'
    with pytest.raises(RoutingError):
        search(g, '00', '20', model, 'bfs')
    for alg in ('astar', 'dijkstra'):
        runs = {tuple(search(lattice(), '00', '33', CostModel(1., 1.2857), alg).nodes) for _ in range(5)}
        assert len(runs) == 1
    oneway = build({'a': (0, 0), 'b': (1, 0), 'c': (1, 1)}, [('a', 'b'), ('b', 'c'), ('a', 'c')], one_way={('a', 'b')})
    assert search(oneway, 'a', 'b', model).nodes == ['a', 'b']
    assert search(oneway, 'b', 'a', model).nodes == ['b', 'c', 'a']


def test_structural_validation_detects_defects():
    g = lattice(2)
    assert validate_structure(g) == ([], [])
    bad = lattice(2)
    bad.edges['00->10'] = Edge('00->10', '00', '10', '00--10', 'corridor', 'c', 1., -1., 3.)
    bad.add_edge(Edge('x', '00', '00', 'loop', 'corridor', 'c', 0., 0., 3., 'one_way'))
    bad.add_edge(Edge('y', '01', 'ghost', 'g', 'corridor', 'c', 1., 1., 3., 'one_way'))
    del bad.edges['11->01']
    errors, _ = validate_structure(bad)
    text = ' '.join(errors)
    for expected in ('negative', 'self-loop', 'endpoint missing', 'lacks a matching reverse'):
        assert expected in text
    with pytest.raises(ValueError):
        bad.add_node(Node('00', 'INTERSECTION', (0, 0)))
    split = build({'a': (0, 0), 'b': (1, 0), 'c': (5, 5), 'd': (6, 5)}, [('a', 'b'), ('c', 'd')])
    assert components(split) == [['a', 'b'], ['c', 'd']]
    assert 'Disconnected' in validate_structure(split)[1][0]
    station = build({'a': (0, 0)}, [])
    station.add_node(Node('s', 'STATION', (1, 0), entity='s'))
    errors, _ = validate_structure(station, [{'id': 's', 'dock_node': 's'}])
    assert any('no station access edge' in e for e in errors) and any('isolated' in e for e in errors)


def test_restrictions_are_hard_and_scenario_specific():
    artifact = {'zones': [{'id': 'blk', 'kind': 'temporary_blocked', 'enabled': None, 'scenario': 'corridor_blocked'},
                          {'id': 'elec', 'kind': 'restricted', 'enabled': False, 'scenario': None}],
                'nodes': [{'id': 'a', 'status': 'validated'}, {'id': 'b', 'status': 'unmapped'}],
                'edges': [{'id': 'a->b', 'connection': 'a--b', 'status': 'validated', 'zones': ['blk']},
                          {'id': 'b->a', 'connection': 'a--b', 'status': 'validated', 'zones': ['elec']}]}
    edges, nodes = exclusions(artifact, Restrictions('normal'))
    assert edges == {} and nodes == {'b': 'map:unmapped'}
    edges, _ = exclusions(artifact, Restrictions('corridor_blocked'))
    assert edges == {'a->b': 'zone:blk'}
    artifact['zones'][1]['enabled'] = True
    edges, _ = exclusions(artifact, Restrictions('normal', frozenset({'a--b'})))
    assert edges == {'a->b': 'blocked', 'b->a': 'blocked'}
    assert exclusions(artifact, Restrictions(require_validated=False))[1] == {}
