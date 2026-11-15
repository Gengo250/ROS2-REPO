/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "1.0";
export type MessageType = "command_result";
export type Accepted = boolean;
export type Reason = string | null;
export type Id = string;
export type Kind = "cone" | "box" | "pallet";
export type Origin = "user";
export type X = number;
export type Y = number;
export type Yaw = number;
export type Length = number;
export type Width = number;
export type Height = number;
export type Removed = number | null;
export type Id1 = string;
export type RobotId = string;
export type Destination = string;
export type Strategy = "direct";
export type Status =
  "pending" | "waiting" | "active" | "recovering" | "succeeded" | "failed" | "canceled" | "rejected" | "blocked";
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
export type WaitingFor = ("station" | "corridor" | "robot") | null;
export type WaitingOn = string | null;
export type BlockingRobot = string | null;
export type QueuePosition = number | null;
export type Yields = number;

export interface CommandResult {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  accepted: Accepted;
  reason: Reason;
  obstacle?: WorldObstacle | null;
  removed?: Removed;
  mission?: Mission | null;
}
export interface WorldObstacle {
  id: Id;
  kind: Kind;
  origin: Origin;
  x: X;
  y: Y;
  yaw: Yaw;
  length: Length;
  width: Width;
  height: Height;
}
/**
 * One robot's mission. ``waiting``: accepted but held by fleet coordination (no Nav2 goal running):
 * queued for its station, before a narrow corridor another AMR is using, or yielding to another AMR
 * (``waiting_for`` / ``waiting_on`` / ``blocking_robot`` / ``queue_position`` say which).
 * ``recovering``: Nav2 recovery behaviours under way. ``blocked``: stopped by the coordinator after a
 * persistent conflict it could not resolve safely (intervention needed; the robot did not arrive).
 */
export interface Mission {
  id: Id1;
  robot_id: RobotId;
  destination: Destination;
  strategy: Strategy;
  status: Status;
  detail: Detail;
  goal: Goal;
  distance_remaining_m: DistanceRemainingM;
  recoveries: Recoveries;
  created_at: CreatedAt;
  updated_at: UpdatedAt;
  waiting_for?: WaitingFor;
  waiting_on?: WaitingOn;
  blocking_robot?: BlockingRobot;
  queue_position?: QueuePosition;
  yields?: Yields;
}
