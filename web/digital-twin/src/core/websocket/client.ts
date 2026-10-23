import { parseTwin, parseTwinV2 } from '../types/validation';
import type { TwinStore } from '../state/store';

export class TwinSocket {
  private socket: WebSocket | null = null;
  private retry: ReturnType<typeof setTimeout> | undefined;
  private watchdog: ReturnType<typeof setTimeout> | undefined;
  private attempts = 0;
  private stopped = true;
  constructor(
    private readonly url: string,
    private readonly store: TwinStore,
    private readonly factory: (url: string) => WebSocket = (url) => new WebSocket(url),
    private readonly random: () => number = Math.random,
  ) {}

  start() {
    if (!this.stopped) return;
    this.stopped = false;
    this.connect();
  }
  private connect() {
    if (this.stopped) return;
    this.store.setConnection('connecting');
    try {
      const socket = this.factory(this.url);
      this.socket = socket;
      socket.onopen = () => this.armWatchdog(15000);
      socket.onmessage = (event) => {
        try {
          const state = (this.url.includes('/ws/v2/') ? parseTwinV2 : parseTwin)(JSON.parse(String(event.data)));
          this.store.accept(state);
          this.attempts = 0;
          this.armWatchdog(Math.max(5000, 3000 / state.connection.configured_publication_hz));
        } catch {
          this.fail('Incompatible or invalid telemetry');
        }
      };
      socket.onerror = () => this.fail('Gateway connection failed');
      socket.onclose = () => this.fail('Gateway disconnected');
      this.armWatchdog(15000);
    } catch { this.fail('Unable to connect to Gateway'); }
  }
  private armWatchdog(milliseconds: number) {
    clearTimeout(this.watchdog);
    this.watchdog = setTimeout(() => this.fail('Telemetry heartbeat timed out'), milliseconds);
  }
  private detach() {
    clearTimeout(this.watchdog);
    if (this.socket) {
      this.socket.onopen = this.socket.onmessage = this.socket.onerror = this.socket.onclose = null;
      this.socket.close();
      this.socket = null;
    }
  }
  private fail(message: string) {
    this.detach();
    if (this.stopped || this.retry !== undefined) return;
    this.store.setConnection('disconnected', message);
    const delay = Math.min(10000, 500 * 2 ** Math.min(this.attempts++, 5)) * (0.8 + 0.2 * this.random());
    this.retry = setTimeout(() => {
      this.retry = undefined;
      this.connect();
    }, delay);
  }
  stop() {
    this.stopped = true;
    clearTimeout(this.retry);
    this.retry = undefined;
    this.detach();
    this.store.setConnection('disconnected');
  }
}
