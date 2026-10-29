/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "1.0";
export type MessageType = "route_status_snapshot";
export type Availability = "unknown" | "offline" | "stale" | "online";
export type ReceivedAt = string | null;
export type AgeSeconds = number | null;
export type ProtocolVersion1 = "1.0";
export type MessageType1 = "route_status";
export type RouteId = string | null;
export type Status =
  | "IDLE"
  | "PLANNING"
  | "PLANNED"
  | "VALIDATING"
  | "EXECUTING"
  | "REPLANNING"
  | "COMPLETED"
  | "BLOCKED"
  | "FAILED"
  | "CANCELED";
export type Origin = string;
export type Destination = string;
export type Algorithm = "astar" | "dijkstra";
export type GraphVersion = string | null;
export type MapVersion = string | null;
export type RegistrationId = string | null;
/**
 * @maxItems 512
 */
export type SelectedNodes = string[];
/**
 * @maxItems 512
 */
export type SelectedEdges = string[];
/**
 * @maxItems 512
 */
export type MapFramePath = [number, number][];
/**
 * @maxItems 512
 */
export type MapFrameWaypoints = [number, number, number][];
/**
 * @maxItems 512
 */
export type WaypointNodes = string[];
export type CurrentWaypoint = number | null;
export type CompletedWaypoints = number;
export type TotalWaypoints = number;
export type EstimatedDistanceM = number | null;
export type DistanceRemainingM = number | null;
export type ExecutionProgress = number;
export type Replans = number;
/**
 * @maxItems 512
 */
export type BlockedConnections = string[];
export type NavigationActionStatus = string | null;
export type FailureReason = string | null;
export type Detail = string | null;
export type Stamp = number;
export type UpdatedAt = string;

export interface RouteStatus {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  availability: Availability;
  received_at: ReceivedAt;
  age_seconds: AgeSeconds;
  route: RouteExecution | null;
}
/**
 * Route executor message on /route_planner/status; map-frame geometry only.
 */
export interface RouteExecution {
  protocol_version: ProtocolVersion1;
  message_type: MessageType1;
  route_id: RouteId;
  status: Status;
  origin: Origin;
  destination: Destination;
  algorithm: Algorithm;
  graph_version: GraphVersion;
  map_version: MapVersion;
  registration_id: RegistrationId;
  selected_nodes: SelectedNodes;
  selected_edges: SelectedEdges;
  map_frame_path: MapFramePath;
  map_frame_waypoints: MapFrameWaypoints;
  waypoint_nodes: WaypointNodes;
  current_waypoint: CurrentWaypoint;
  completed_waypoints: CompletedWaypoints;
  total_waypoints: TotalWaypoints;
  estimated_distance_m: EstimatedDistanceM;
  distance_remaining_m: DistanceRemainingM;
  execution_progress: ExecutionProgress;
  replans: Replans;
  blocked_connections: BlockedConnections;
  navigation_action_status: NavigationActionStatus;
  failure_reason: FailureReason;
  detail: Detail;
  stamp: Stamp;
  updated_at: UpdatedAt;
}
