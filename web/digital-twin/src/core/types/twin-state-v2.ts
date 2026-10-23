/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "2.0";
export type MessageType = "twin_state";
export type SessionId = string;
export type Sequence = number;
export type Timestamp = string;
export type SimulationTime = number | null;
export type Epoch = number;
export type Status = "unknown" | "running" | "paused_or_stalled" | "offline";
export type PoseTopic = string;
export type Id = string;
export type FrameId = string;
export type Scenario = ("normal" | "person_crossing" | "obstacle_in_aisle" | "corridor_blocked") | null;
export type ScenarioSource = "launch_configuration" | "environment" | "unspecified";
export type Status1 = "unknown" | "running" | "unavailable";
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
export type Status2 = "unknown" | "offline" | "stale" | "online";
export type SourceTimestamp = number | null;
export type ReceivedAt = string | null;
export type AgeSeconds = number | null;
export type Robots = Robot[];
export type Id2 = string;
export type Type = "robot" | "human" | "obstacle";
export type GazeboName = string;
export type FrameId2 = string;
export type SourceTimestamp1 = number | null;
export type ReceivedAt1 = string | null;
export type AgeSeconds1 = number | null;
export type Status3 = "unknown" | "offline" | "stale" | "online";
export type PoseSource1 = string;
export type Id3 = string;
export type FrameId3 = string;
/**
 * @maxItems 64
 */
export type Components = EntityTransform[];
export type SimulationRobots = DynamicEntity[];
export type Humans = DynamicEntity[];
export type Obstacles = DynamicEntity[];
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
export type Status4 = "unknown" | "offline" | "stale" | "online";
export type ReceivedAt2 = string | null;
export type AgeSeconds2 = number | null;
export type MessagesReceived = number;
export type Topics = TopicStatus[];

export interface TwinStateV2 {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  session_id: SessionId;
  sequence: Sequence;
  timestamp: Timestamp;
  simulation_time: SimulationTime;
  simulation: SimulationState;
  world: World;
  robots: Robots;
  simulation_robots: SimulationRobots;
  humans: Humans;
  obstacles: Obstacles;
  stations?: Stations;
  connection: ConnectionStatus;
}
export interface SimulationState {
  epoch: Epoch;
  status: Status;
  pose_topic: PoseTopic;
}
export interface World {
  id: Id;
  frame_id: FrameId;
  scenario: Scenario;
  scenario_source: ScenarioSource;
  status: Status1;
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
  status: Status2;
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
export interface DynamicEntity {
  id: Id2;
  type: Type;
  gazebo_name: GazeboName;
  frame_id: FrameId2;
  pose: Pose | null;
  velocity?: Velocity | null;
  source_timestamp: SourceTimestamp1;
  received_at: ReceivedAt1;
  age_seconds: AgeSeconds1;
  status: Status3;
  pose_source: PoseSource1;
  components?: Components;
}
export interface EntityTransform {
  id: Id3;
  frame_id: FrameId3;
  pose: Pose;
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
  status: Status4;
  received_at: ReceivedAt2;
  age_seconds: AgeSeconds2;
  messages_received: MessagesReceived;
}
