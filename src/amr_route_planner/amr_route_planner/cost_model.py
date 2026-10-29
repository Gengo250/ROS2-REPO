"""Explicit route cost and restriction model; independent of the search algorithm.

Cost unit: metres-equivalent of travel. An edge costs its ``base_cost``
(``distance_weight · distance`` plus configured nonnegative operational
penalties, fixed in the artifact). Changing heading at a node adds
``turn_weight_m_per_rad · |Δheading|`` (in-place rotation time expressed as
forgone travel). Hard restrictions never become large costs: the edge is removed.
Corridor capacity is design metadata and is not used as live occupancy.
"""
from dataclasses import dataclass, field
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class CostModel:
    distance_weight: float = 1.0
    turn_weight: float = 0.0

    @classmethod
    def from_settings(cls, settings):
        cost = settings['cost']
        return cls(cost['distance_weight'], cost['turn_weight_m_per_rad'])

    def edge(self, edge):
        return edge.base_cost

    def turn(self, heading_in, heading_out):
        if heading_in is None or heading_out is None:
            return 0.
        return self.turn_weight*abs(wrap(heading_out-heading_in))

    def lower_bound(self, a, b):
        """Admissible A* heuristic: every edge costs at least distance_weight × its straight length."""
        return self.distance_weight*math.dist(a, b)


@dataclass(frozen=True)
class Restrictions:
    """Operational exclusions applied at planning time."""
    scenario: str | None = None
    blocked_connections: frozenset = field(default_factory=frozenset)
    unavailable_nodes: frozenset = field(default_factory=frozenset)
    require_validated: bool = True


def active_zones(artifact, restrictions):
    zones = set()
    for zone in artifact['zones']:
        if zone['kind'] == 'restricted' and zone['enabled']:
            zones.add(zone['id'])
        elif zone['kind'] == 'temporary_blocked' and zone['scenario'] == restrictions.scenario:
            zones.add(zone['id'])
    return zones


def exclusions(artifact, restrictions):
    """Edge and node ids that must not be traversed, each with its reason."""
    zones = active_zones(artifact, restrictions)
    nodes, edges = {}, {}
    for n in artifact['nodes']:
        if restrictions.require_validated and n['status'] != 'validated':
            nodes[n['id']] = 'map:'+n['status']
        if n['id'] in restrictions.unavailable_nodes:
            nodes[n['id']] = 'unavailable'
    for e in artifact['edges']:
        if restrictions.require_validated and e['status'] != 'validated':
            edges[e['id']] = 'map:'+e['status']
        hit = sorted(zones & set(e['zones']))
        if hit:
            edges[e['id']] = 'zone:'+','.join(hit)
        if e['connection'] in restrictions.blocked_connections:
            edges[e['id']] = 'blocked'
    return edges, nodes
