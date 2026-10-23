import Ajv2020 from 'ajv/dist/2020';
import addFormats from 'ajv-formats';
import twinSchema from '../../../../../shared/schemas/twin-state.schema.json';
import twinV2Schema from '../../../../../shared/schemas/twin-state-v2.schema.json';
import type { TwinStateV2 } from './twin-state-v2';
import statusSchema from '../../../../../shared/schemas/connection-status.schema.json';
import type { TwinState } from './twin-state';
import type { ConnectionStatus } from './connection-status';

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
