# Digital Twin web diagnostics

Lightweight TypeScript + Vite application using the native browser WebSocket API.
Three.js is installed for Phase 2; no renderer or warehouse scene is created here.

Use Node >=22.12, then `npm ci`, `npm run dev` and open http://127.0.0.1:5173.
See [.env.example](.env.example) for endpoints. The page works with no backend and
never generates fake telemetry. It shows last known values explicitly after a
connection loss. Backend payloads are validated against the shared JSON Schemas;
TypeScript types are generated with `npm run generate:types`.

`npm test` runs client/protocol tests; `npm run build` checks schema type drift,
strict TypeScript and production bundling. `npm run test:browser` requires a running
Vite server. For real Gazebo → browser validation, use the Python harness documented
in [development](../../docs/digital-twin/DEVELOPMENT.md). It also validates motion,
multiple clients, network loss/reconnection and shutdown.
