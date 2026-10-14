/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "1.0";
export type MessageType = "twin_state";
export type SessionId = string;
export type Sequence = number;
export type Timestamp = string;
export type SimulationTime = number | null;
export type Id = string;
export type FrameId = string;
export type Scenario = ("normal" | "person_crossing" | "obstacle_in_aisle" | "corridor_blocked") | null;
export type ScenarioSource = "launch_configuration" | "environment" | "unspecified";
export type Status = "unknown" | "running" | "unavailable";
export type Availability = "unknown" | "offline" | "stale" | "online";
export type StatusSource = "clock_progress";
export type Id1 = string;
export type FrameId1 = string | null;
export type ChildFrameId = string | null;
export type PoseSource = string;
export type X = number;
export type Y = number;
export type Z = number;
export type X1 = number;
export type Y1 = number;
export type Z1 = number;
export type W = number;
export type Status1 = "unknown" | "offline" | "stale" | "online";
export type SourceTimestamp = number | null;
export type ReceivedAt = string | null;
export type AgeSeconds = number | null;
export type Robots = Robot[];
/**
 * @minItems 0
 * @maxItems 0
 */
export type Humans = [];
/**
 * @minItems 0
 * @maxItems 0
 */
export type Obstacles = [];
/**
 * @minItems 0
 * @maxItems 0
 */
export type Stations = [];
export type ProtocolVersion1 = "1.0";
export type MessageType1 = "connection_status";
export type SessionId1 = string;
export type Gateway = "online" | "offline";
export type RosInitialized = boolean;
export type Ros = "unknown" | "offline" | "stale" | "online";
export type Simulation = "unknown" | "offline" | "stale" | "online";
export type LastOdometryTimestamp = string | null;
export type ActiveWebsocketClients = number;
export type ConfiguredPublicationHz = number;
export type MeasuredPublicationHz = number;
export type Topic = string;
export type Status2 = "unknown" | "offline" | "stale" | "online";
export type ReceivedAt1 = string | null;
export type AgeSeconds1 = number | null;
export type MessagesReceived = number;
export type Topics = TopicStatus[];

export interface TwinState {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  session_id: SessionId;
  sequence: Sequence;
  timestamp: Timestamp;
  simulation_time: SimulationTime;
  world: World;
  robots: Robots;
  humans?: Humans;
  obstacles?: Obstacles;
  stations?: Stations;
  connection: ConnectionStatus;
}
export interface World {
  id: Id;
  frame_id: FrameId;
  scenario: Scenario;
  scenario_source: ScenarioSource;
  status: Status;
  availability: Availability;
  status_source?: StatusSource;
}
export interface Robot {
  id: Id1;
  frame_id: FrameId1;
  child_frame_id: ChildFrameId;
  pose_source: PoseSource;
  pose: Pose | null;
  velocity: Velocity | null;
  status: Status1;
  source_timestamp: SourceTimestamp;
  received_at: ReceivedAt;
  age_seconds: AgeSeconds;
}
export interface Pose {
  position: Vector3;
  orientation: Quaternion;
}
export interface Vector3 {
  x: X;
  y: Y;
  z: Z;
}
export interface Quaternion {
  x: X1;
  y: Y1;
  z: Z1;
  w: W;
}
export interface Velocity {
  linear: Vector3;
  angular: Vector3;
}
export interface ConnectionStatus {
  protocol_version?: ProtocolVersion1;
  message_type?: MessageType1;
  session_id: SessionId1;
  gateway: Gateway;
  ros_initialized: RosInitialized;
  ros: Ros;
  simulation: Simulation;
  last_odometry_timestamp: LastOdometryTimestamp;
  active_websocket_clients: ActiveWebsocketClients;
  configured_publication_hz: ConfiguredPublicationHz;
  measured_publication_hz: MeasuredPublicationHz;
  topics: Topics;
}
export interface TopicStatus {
  topic: Topic;
  status: Status2;
  received_at: ReceivedAt1;
  age_seconds: AgeSeconds1;
  messages_received: MessagesReceived;
}
