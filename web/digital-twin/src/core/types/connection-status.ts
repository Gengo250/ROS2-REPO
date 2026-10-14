/* Generated from shared/schemas. Run npm run generate:types. */

export type ProtocolVersion = "1.0";
export type MessageType = "connection_status";
export type SessionId = string;
export type Gateway = "online" | "offline";
export type RosInitialized = boolean;
export type Ros = "unknown" | "offline" | "stale" | "online";
export type Simulation = "unknown" | "offline" | "stale" | "online";
export type LastOdometryTimestamp = string | null;
export type ActiveWebsocketClients = number;
export type ConfiguredPublicationHz = number;
export type MeasuredPublicationHz = number;
export type Topic = string;
export type Status = "unknown" | "offline" | "stale" | "online";
export type ReceivedAt = string | null;
export type AgeSeconds = number | null;
export type MessagesReceived = number;
export type Topics = TopicStatus[];

export interface ConnectionStatus {
  protocol_version?: ProtocolVersion;
  message_type?: MessageType;
  session_id: SessionId;
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
  status: Status;
  received_at: ReceivedAt;
  age_seconds: AgeSeconds;
  messages_received: MessagesReceived;
}
