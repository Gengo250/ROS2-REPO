/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "1.0";
export type MessageType = "world_state";
export type SessionId = string;
export type Sequence = number;
export type Timestamp = string;
export type ServerWallTime = number;
export type Backend = "lite" | "gazebo" | "none";
export type Status = "unknown" | "running" | "paused" | "stalled" | "offline";
export type Availability = "unknown" | "offline" | "stale" | "online";
export type EngineSessionId = string | null;
export type SimTime = number | null;
export type RealTimeFactor = number | null;
export type TargetRealTimeFactor = number | null;
export type Overruns = number;
export type SceneSha256 = string | null;
export type AgeSeconds = number | null;
export type Gateway = "online";
export type Ros = "unknown" | "offline" | "stale" | "online";
export type Simulation1 = "unknown" | "offline" | "stale" | "online";
export type Navigation = "unknown" | "offline" | "stale" | "online";
export type Registration = "verified" | "missing";
export type Id = string;
export type Type = "amr" | "human" | "forklift";
export type X = number;
export type Y = number;
export type Z = number;
export type Yaw = number;
export type Vx = number;
export type Vy = number;
export type Wz = number;
export type Speed = number;
export type State = string;
export type Activity = string | null;
export type ActivityProgress = number;
export type Carrying = boolean;
export type Look = number;
export type FieldState = "free" | "warning" | "protective";
export type FieldLength = number;
export type Reason = string | null;
/**
 * @maxItems 256
 */
export type Entities = WorldEntity[];
/**
 * @maxItems 16
 */
export type Robots =
  | []
  | [RobotNavigation]
  | [RobotNavigation, RobotNavigation]
  | [RobotNavigation, RobotNavigation, RobotNavigation]
  | [RobotNavigation, RobotNavigation, RobotNavigation, RobotNavigation]
  | [RobotNavigation, RobotNavigation, RobotNavigation, RobotNavigation, RobotNavigation]
  | [RobotNavigation, RobotNavigation, RobotNavigation, RobotNavigation, RobotNavigation, RobotNavigation]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ]
  | [
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation,
      RobotNavigation
    ];
export type Id1 = string;
export type Interlock = string | null;
export type InterlockReady = boolean;
export type InterlockAvailability = "unknown" | "offline" | "stale" | "online";
/**
 * @maxItems 256
 */
export type Path = [number, number][];
export type PathAgeSeconds = number | null;
export type PathFrame = "world";
export type PathOmittedReason = string | null;
export type Id2 = string;
export type RobotId = string;
export type Destination = string;
export type Strategy = "direct";
export type Status1 = "pending" | "active" | "succeeded" | "failed" | "canceled" | "rejected";
export type Detail = string | null;
/**
 * @minItems 3
 * @maxItems 3
 */
export type Goal = [number, number, number];
export type DistanceRemainingM = number | null;
export type Recoveries = number;
export type CreatedAt = string;
export type UpdatedAt = string;
export type Id3 = string;
export type Kind = "cone" | "box" | "pallet";
export type Origin = "user";
export type X1 = number;
export type Y1 = number;
export type Yaw1 = number;
export type Length = number;
export type Width = number;
export type Height = number;
/**
 * @maxItems 256
 */
export type Obstacles = WorldObstacle[];
export type Seq = number;
export type SimTime1 = number;
export type Kind1 = string;
export type Entity = string;
export type Other = string;
export type Detail1 = string;
export type X2 = number;
export type Y2 = number;
/**
 * @maxItems 50
 */
export type Events = WorldEvent[];

export interface WorldState {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  session_id: SessionId;
  sequence: Sequence;
  timestamp: Timestamp;
  server_wall_time: ServerWallTime;
  simulation: Simulation;
  connection: Connection;
  entities: Entities;
  robots: Robots;
  obstacles: Obstacles;
  events: Events;
}
export interface Simulation {
  backend: Backend;
  status: Status;
  availability: Availability;
  engine_session_id: EngineSessionId;
  sim_time: SimTime;
  real_time_factor: RealTimeFactor;
  target_real_time_factor: TargetRealTimeFactor;
  overruns: Overruns;
  scene_sha256: SceneSha256;
  age_seconds: AgeSeconds;
}
export interface Connection {
  gateway: Gateway;
  ros: Ros;
  simulation: Simulation1;
  navigation: Navigation;
  registration: Registration;
}
export interface WorldEntity {
  id: Id;
  type: Type;
  x: X;
  y: Y;
  z: Z;
  yaw: Yaw;
  vx: Vx;
  vy: Vy;
  wz: Wz;
  speed: Speed;
  state: State;
  activity: Activity;
  activity_progress: ActivityProgress;
  carrying: Carrying;
  look: Look;
  field_state: FieldState;
  field_length: FieldLength;
  reason: Reason;
}
export interface RobotNavigation {
  id: Id1;
  interlock: Interlock;
  interlock_ready: InterlockReady;
  interlock_availability: InterlockAvailability;
  path: Path;
  path_age_seconds: PathAgeSeconds;
  path_frame: PathFrame;
  path_omitted_reason: PathOmittedReason;
  mission: Mission | null;
}
export interface Mission {
  id: Id2;
  robot_id: RobotId;
  destination: Destination;
  strategy: Strategy;
  status: Status1;
  detail: Detail;
  goal: Goal;
  distance_remaining_m: DistanceRemainingM;
  recoveries: Recoveries;
  created_at: CreatedAt;
  updated_at: UpdatedAt;
}
export interface WorldObstacle {
  id: Id3;
  kind: Kind;
  origin: Origin;
  x: X1;
  y: Y1;
  yaw: Yaw1;
  length: Length;
  width: Width;
  height: Height;
}
export interface WorldEvent {
  seq: Seq;
  sim_time: SimTime1;
  kind: Kind1;
  entity: Entity;
  other: Other;
  detail: Detail1;
  x: X2;
  y: Y2;
}
