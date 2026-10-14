import type { TwinState } from '../types/twin-state';
import type { ConnectionStatus } from '../types/connection-status';

export type Connection = 'connecting' | 'connected' | 'disconnected';
export class TwinStore {
  connection: Connection = 'disconnected';
  twin: TwinState | null = null;
  status: ConnectionStatus | null = null;
  error = '';
  private listeners = new Set<() => void>();
  subscribe(listener: () => void) {
    this.listeners.add(listener);
    return () => { this.listeners.delete(listener); };
  }
  setConnection(value: Connection, error = '') {
    this.connection = value;
    this.error = error;
    this.emit();
  }
  setStatus(value: ConnectionStatus) { this.status = value; this.emit(); }
  accept(value: TwinState) {
    if (this.twin?.session_id === value.session_id && value.sequence <= this.twin.sequence) return;
    this.twin = value;
    this.status = value.connection;
    this.connection = 'connected';
    this.error = '';
    this.emit();
  }
  private emit() { for (const listener of this.listeners) listener(); }
}
