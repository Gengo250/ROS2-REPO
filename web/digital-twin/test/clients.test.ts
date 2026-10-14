import { afterEach, describe, expect, it, vi } from 'vitest';
import { TwinStore } from '../src/core/state/store';
import { TwinSocket } from '../src/core/websocket/client';
import { ApiClient } from '../src/core/api/client';
import { parseTwin } from '../src/core/types/validation';
import type { TwinState } from '../src/core/types/twin-state';

function state(sequence = 1): TwinState {
  return { protocol_version: '1.0', message_type: 'twin_state', session_id: 'test-session',
    sequence, timestamp: '2026-10-13T00:00:00Z', simulation_time: null,
    world: { id: 'warehouse', frame_id: 'world', scenario: null, scenario_source: 'unspecified',
      status: 'unknown', availability: 'unknown', status_source: 'clock_progress' },
    robots: [], humans: [], obstacles: [], stations: [],
    connection: { protocol_version: '1.0', message_type: 'connection_status', session_id: 'test-session',
      gateway: 'online', ros_initialized: true, ros: 'online', simulation: 'unknown',
      last_odometry_timestamp: null, active_websocket_clients: 1,
      configured_publication_hz: 10, measured_publication_hz: 10, topics: [] } };
}
class Socket {
  onopen: (() => void) | null = null;
  onmessage: ((event: { data: string }) => void) | null = null;
  onerror: (() => void) | null = null;
  onclose: (() => void) | null = null;
  close = vi.fn();
}
afterEach(() => vi.useRealTimers());
describe('shared protocol', () => {
  it('validates messages and rejects wrong versions and malformed fields', () => {
    expect(parseTwin(state()).sequence).toBe(1);
    expect(() => parseTwin({ ...state(), protocol_version: '2.0' })).toThrow();
    expect(() => parseTwin({ ...state(), sequence: -1 })).toThrow();
    expect(() => parseTwin({ ...state(), humans: [{ id: 'invented' }] })).toThrow();
    expect(() => parseTwin({ ...state(), timestamp: 'bad timestamp' })).toThrow();
  });
  it('rejects old sequence within a session, accepts a restarted gateway', () => {
    const store = new TwinStore();
    store.accept(state(5)); store.accept(state(2));
    expect(store.twin?.sequence).toBe(5);
    store.accept({ ...state(1), session_id: 'new-session' });
    expect(store.twin?.sequence).toBe(1);
  });
});
it('validates HTTP data and surfaces failures', async () => {
  const fetcher = vi.fn().mockResolvedValue(new Response(JSON.stringify(state())));
  const api = new ApiClient('http://127.0.0.1:8000', fetcher);
  expect((await api.state()).sequence).toBe(1);
  fetcher.mockResolvedValue(new Response('', { status: 503 }));
  await expect(api.status()).rejects.toThrow('503');
  fetcher.mockResolvedValue(new Response('{}'));
  await expect(api.state()).rejects.toThrow('Invalid TwinState');
});
it('connects, retries with bounded backoff, resets after data and cleans up', () => {
  vi.useFakeTimers();
  const sockets: Socket[] = [];
  const store = new TwinStore();
  const create = vi.fn(() => { const s = new Socket(); sockets.push(s); return s as unknown as WebSocket; });
  const client = new TwinSocket('ws://localhost', store, create, () => 1);
  client.start(); client.start();
  expect(create).toHaveBeenCalledTimes(1);
  expect(store.connection).toBe('connecting');
  sockets[0]!.onmessage!({ data: JSON.stringify(state()) });
  expect(store.connection).toBe('connected');
  sockets[0]!.onclose!();
  expect(store.connection).toBe('disconnected');
  for (const delay of [500, 1000, 2000, 4000, 8000, 10000, 10000]) {
    const before = create.mock.calls.length;
    vi.advanceTimersByTime(delay - 1);
    expect(create).toHaveBeenCalledTimes(before);
    vi.advanceTimersByTime(1);
    expect(create).toHaveBeenCalledTimes(before + 1);
    sockets.at(-1)!.onclose!();
  }
  client.stop();
  vi.advanceTimersByTime(60000);
  expect(vi.getTimerCount()).toBe(0);
  expect(sockets.every((s) => s.onmessage === null)).toBe(true);
});
it('detects a silent half-open socket and rejects invalid payloads', () => {
  vi.useFakeTimers();
  const store = new TwinStore();
  const sockets: Socket[] = [];
  const client = new TwinSocket('ws://localhost', store, () => {
    const s = new Socket(); sockets.push(s); return s as unknown as WebSocket;
  }, () => 1);
  client.start(); sockets[0]!.onopen!();
  vi.advanceTimersByTime(15000);
  expect(store.connection).toBe('disconnected');
  vi.advanceTimersByTime(500);
  sockets[1]!.onmessage!({ data: '{invalid' });
  expect(store.error).toContain('invalid');
  client.stop();
});
it('resets retry delay after a successful reconnection and unsubscribes listeners', () => {
  vi.useFakeTimers();
  const store = new TwinStore();
  const listener = vi.fn();
  const unsubscribe = store.subscribe(listener);
  const sockets: Socket[] = [];
  const client = new TwinSocket('ws://localhost', store, () => {
    const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket;
  }, () => 1);
  client.start(); sockets[0]!.onclose!();
  vi.advanceTimersByTime(500); sockets[1]!.onclose!();
  vi.advanceTimersByTime(1000);
  sockets[2]!.onmessage!({ data: JSON.stringify(state()) });
  expect(store.connection).toBe('connected');
  sockets[2]!.onclose!();
  vi.advanceTimersByTime(500);
  expect(sockets).toHaveLength(4);
  unsubscribe(); const calls = listener.mock.calls.length;
  client.stop();
  expect(listener).toHaveBeenCalledTimes(calls);
});
