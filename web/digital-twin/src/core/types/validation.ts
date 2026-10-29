import Ajv2020 from 'ajv/dist/2020';
import addFormats from 'ajv-formats';
import twinSchema from '../../../../../shared/schemas/twin-state.schema.json';
import twinV2Schema from '../../../../../shared/schemas/twin-state-v2.schema.json';
import type { TwinStateV2 } from './twin-state-v2';
import statusSchema from '../../../../../shared/schemas/connection-status.schema.json';
import type { TwinState } from './twin-state';
import type { ConnectionStatus } from './connection-status';
import routeGraphSchema from '../../../../../shared/schemas/route-graph.v1.schema.json';
import routeStatusSchema from '../../../../../shared/schemas/route-status.v1.schema.json';
import type { RouteGraph } from './route-graph';
import type { RouteStatus } from './route-status';

const ajv = new Ajv2020({ strict: true, allErrors: false });
addFormats(ajv);
const twin = ajv.compile<TwinState>(twinSchema);
const status = ajv.compile<ConnectionStatus>(statusSchema);
export function parseTwin(value: unknown): TwinState {
  if (!twin(value)) throw new Error(`Invalid TwinState: ${ajv.errorsText(twin.errors)}`);
  return value;
}
export function parseStatus(value: unknown): ConnectionStatus {
  if (!status(value)) throw new Error(`Invalid connection status: ${ajv.errorsText(status.errors)}`);
  return value;
}

const twinV2 = ajv.compile<TwinStateV2>(twinV2Schema);
export function parseTwinV2(value: unknown): TwinStateV2 {
  if (!twinV2(value)) throw new Error(`Invalid TwinState v2: ${ajv.errorsText(twinV2.errors)}`);
  return value;
}

const routeGraph = ajv.compile<RouteGraph>(routeGraphSchema);
const routeStatus = ajv.compile<RouteStatus>(routeStatusSchema);
export function parseRouteGraph(value: unknown): RouteGraph {
  if (!routeGraph(value)) throw new Error(`Invalid route graph: ${ajv.errorsText(routeGraph.errors)}`);
  return value;
}
export function parseRouteStatus(value: unknown): RouteStatus {
  if (!routeStatus(value)) throw new Error(`Invalid route status: ${ajv.errorsText(routeStatus.errors)}`);
  return value;
}
