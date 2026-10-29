/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "1.0";
export type MessageType = "route_graph";
export type GraphVersion = string;
export type MapVersion = string;
export type FrameId = "world";
export type Id = string;
export type ParentFrame = "world";
export type ChildFrame = "map";
export type X = number;
export type Y = number;
export type Yaw = number;
export type Valid = boolean;
export type Id1 = string;
export type Type = "STATION" | "INTERSECTION" | "CORRIDOR_POINT" | "OPEN_AREA";
/**
 * @minItems 2
 * @maxItems 2
 */
export type Position = [number, number];
export type Yaw1 = number | null;
export type Status = "validated" | "unmapped" | "blocked" | "invalid";
export type Reason = string;
export type Nodes = RouteGraphNode[];
export type Id2 = string;
export type Source = string;
export type Target = string;
export type Directions = string[];
export type Kind = "corridor" | "station_access";
export type CorridorId = string | null;
export type Width = number;
export type Distance = number;
export type Status1 = "validated" | "unmapped" | "blocked" | "invalid";
export type Zones = string[];
export type Connections = RouteGraphConnection[];
export type Id3 = string;
export type Label = string;
export type DockNode = string;
export type Status2 = "validated" | "unmapped" | "blocked" | "invalid";
export type Reason1 = string;
export type Stations = RouteStation[];

/**
 * Static graph geometry in the world frame; fetched once, never broadcast.
 */
export interface RouteGraph {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  graph_version: GraphVersion;
  map_version: MapVersion;
  frame_id: FrameId;
  registration: RouteRegistration;
  nodes: Nodes;
  connections: Connections;
  stations: Stations;
}
export interface RouteRegistration {
  id: Id;
  parent_frame: ParentFrame;
  child_frame: ChildFrame;
  x: X;
  y: Y;
  yaw: Yaw;
  valid: Valid;
}
export interface RouteGraphNode {
  id: Id1;
  type: Type;
  position: Position;
  yaw: Yaw1;
  status: Status;
  reason: Reason;
}
export interface RouteGraphConnection {
  id: Id2;
  source: Source;
  target: Target;
  directions: Directions;
  kind: Kind;
  corridor_id: CorridorId;
  width: Width;
  distance: Distance;
  status: Status1;
  zones: Zones;
}
export interface RouteStation {
  id: Id3;
  label: Label;
  dock_node: DockNode;
  status: Status2;
  reason: Reason1;
}
