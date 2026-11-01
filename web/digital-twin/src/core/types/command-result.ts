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
export type Status = "pending" | "active" | "succeeded" | "failed" | "canceled" | "rejected";
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
}
