import { parseStatus, parseTwin } from '../types/validation';

export class ApiClient {
  constructor(private readonly base: string, private readonly fetcher: typeof fetch = fetch) {}
  private async get(path: string, signal?: AbortSignal): Promise<unknown> {
    const deadline = AbortSignal.timeout(4000);
    const response = await this.fetcher(new URL(path, this.base), {
      signal: signal ? AbortSignal.any([signal, deadline]) : deadline,
      headers: { Accept: 'application/json' },
    });
    if (!response.ok) throw new Error(`Gateway HTTP ${response.status}`);
    return response.json();
  }
  async status(signal?: AbortSignal) { return parseStatus(await this.get('/api/v1/status', signal)); }
  async state(signal?: AbortSignal) { return parseTwin(await this.get('/api/v1/state', signal)); }
}
