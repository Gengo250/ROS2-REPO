/* Generated from shared/schemas. Run npm run generate:types. */

export type SchemaVersion = "1.0";
export type WorldId = string;
export type Scenario = "normal" | "person_crossing" | "obstacle_in_aisle" | "corridor_blocked";
export type ConfiguredScenario = ("normal" | "person_crossing" | "obstacle_in_aisle" | "corridor_blocked") | null;
export type SourceVersion = number;
export type GeometrySha256 = string;
export type Frame = "world";
export type X = "east";
export type Y = "north";
export type Z = "up";
export type Units = "metres_radians";
export type Rotation = "Rz(yaw) Ry(pitch) Rx(roll)";
export type Length = number;
export type Width = number;
export type Height = number;
/**
 * @minItems 3
 * @maxItems 3
 */
export type FloorSize = [number, number, number];
export type Id = string;
/**
 * @minItems 6
 * @maxItems 6
 */
export type Pose = [number, number, number, number, number, number];
export type Kind = string;
export type PoseSource = "configured_initial_pose";
export type Entities = Entity[];
export type Id1 = string;
export type Kind1 = "box" | "cylinder" | "sphere" | "label";
export type Dimensions = number[];
export type Transforms = [number, number, number, number, number, number][];
/**
 * @minItems 4
 * @maxItems 4
 */
export type Colour = [number, number, number, number];
export type Label = string | null;
export type Category = "structure" | "wall" | "pillar" | "beam" | "floor" | "marking" | "equipment" | "human";
export type Primitives = Primitive[];
export type Id2 = string;
/**
 * @minItems 2
 * @maxItems 2
 */
export type Centre = [number, number];
/**
 * @minItems 2
 * @maxItems 2
 */
export type Size = [number, number];
export type Enabled = boolean;
export type Aisles = Area[];
export type Zones = Area[];
export type Id3 = string;
export type Label1 = string;
/**
 * @minItems 6
 * @maxItems 6
 */
export type Pose1 = [number, number, number, number, number, number];
/**
 * @minItems 6
 * @maxItems 6
 */
export type GoalPose = [number, number, number, number, number, number];
/**
 * @minItems 2
 * @maxItems 2
 */
export type Footprint = [number, number];
export type Stations = Station[];
export type Id4 = string;
export type RobotId = string;
export type Active = boolean;
/**
 * @minItems 6
 * @maxItems 6
 */
export type Pose2 = [number, number, number, number, number, number];
export type RobotSpawns = Spawn[];

export interface WarehouseScene {
  schema_version?: SchemaVersion;
  metadata: Metadata;
  coordinate_system?: Coordinates;
  building: Building;
  entities: Entities;
  primitives: Primitives;
  aisles: Aisles;
  zones: Zones;
  stations: Stations;
  robot_spawns: RobotSpawns;
}
export interface Metadata {
  world_id: WorldId;
  scenario: Scenario;
  configured_scenario: ConfiguredScenario;
  source_version: SourceVersion;
  source_sha256: SourceSha256;
  geometry_sha256: GeometrySha256;
}
export interface SourceSha256 {
  [k: string]: string;
}
export interface Coordinates {
  frame?: Frame;
  x?: X;
  y?: Y;
  z?: Z;
  units?: Units;
  rotation?: Rotation;
}
export interface Building {
  length: Length;
  width: Width;
  height: Height;
  floor_size: FloorSize;
}
export interface Entity {
  id: Id;
  pose: Pose;
  kind: Kind;
  pose_source?: PoseSource;
}
export interface Primitive {
  id: Id1;
  kind: Kind1;
  dimensions: Dimensions;
  transforms: Transforms;
  colour: Colour;
  label?: Label;
  category: Category;
}
export interface Area {
  id: Id2;
  centre: Centre;
  size: Size;
  enabled?: Enabled;
}
export interface Station {
  id: Id3;
  label: Label1;
  pose: Pose1;
  goal_pose: GoalPose;
  footprint: Footprint;
}
export interface Spawn {
  id: Id4;
  robot_id: RobotId;
  active: Active;
  pose: Pose2;
}
