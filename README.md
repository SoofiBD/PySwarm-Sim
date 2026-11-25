# Autonomous UAV Swarm Simulation

A physics-based multi-drone simulation with real-time WebSocket telemetry and live Google Maps visualisation. Architected for production deployment on a Linux VPS behind an nginx TLS reverse proxy.

## Architecture

```
Browser
  │  wss://YOUR_DOMAIN/ws/telemetry  (60 fps interpolated animation)
  │  https://YOUR_DOMAIN/            (index.html)
  │  https://YOUR_DOMAIN/static/**   (JS/CSS — served directly by nginx)
  ▼
nginx:443 ──TLS──► FastAPI / uvicorn :8000
                        │
                        ├─ SimulationRunner (asyncio task, 20 Hz)
                        │       │
                        │       ├─ AStarPathfinder (3D, 26-connected, O(V log V))
                        │       ├─ DronePhysics (PD thrust + gravity + drag, Euler)
                        │       └─ CollisionDetector (scipy cKDTree, O(N log N))
                        │
                        └─ ConnectionManager (WebSocket broadcast to all clients)
```

## Quick Start (Docker)

```bash
git clone https://github.com/SoofiBD/PythonDrone-master
cd PythonDrone-master
cp .env.example .env
# (Optional) Edit .env: set MAPS_API_KEY to your Google Maps JS API key
docker compose up --build
```

Open `http://localhost` in a browser.

## Quick Start (local dev)

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e ".[test]"          # add ,desktop for the PySide6 UI (python run.py)
uvicorn server.main:app --reload
```

Open `http://localhost:8000`.

## Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `ADMIN_TOKEN` | Yes (Docker) | — | Required in `X-Admin-Token` header for `POST /api/*`; if unset (local dev) those endpoints are open |
| `MAPS_API_KEY` | No | — | Google Maps JavaScript API key (if omitted, map loads in development mode) |
| `N_DRONES` | No | `6` | Number of drones at startup |
| `N_TARGETS` | No | `8` | Number of targets at startup |
| `SIM_DT` | No | `0.05` | Simulation timestep (seconds) |
| `CTHR` | No | `200.0` | Communication range threshold (metres) |
| `SWARM_KEY` | No | dev key | Hex HMAC key for drone packets. If unset or empty a built-in development key is used and a warning is logged |
| `COMM_MODE` | No | `ideal` | Startup comm mode: `ideal`, `fixed_rate` or `predictive`. An invalid value falls back to `ideal`. `POST /api/config` overrides it on restart |

## REST API

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/status` | WebSocket client count + simulation running state |
| `GET` | `/api/metrics` | Step count, collisions resolved, targets visited |
| `POST` | `/api/control` | `{"action": "start" \| "stop" \| "reset"}` |
| `POST` | `/api/config` | `{"n_drones": N, "n_targets": M}` — reconfigures and restarts |
| `POST` | `/api/config` | also accepts `"comm_mode": "ideal" \| "fixed_rate" \| "predictive"` (default `ideal`) |
| `POST` | `/api/fault` | `{"kind": "gps_spoof" \| "replay", "drone_id": N \| null}` — injects a fault, needs `X-Admin-Token`. 409 in `ideal` mode, 404 for an unknown drone. Replay transmits from the swarm centroid |

When comm mode is not `ideal`, `/api/metrics` also returns a `comm` object and each telemetry drone carries `trust`, `quarantined` and `suspect`. The map shows quarantined drones in red and suspect ones in orange.

## Drone-to-drone communication

In `fixed_rate` and `predictive` mode, drones no longer know each other's true positions. Each drone broadcasts a 32-byte packet, and neighbours, link state and the connectivity graph are built from what was actually received. `ideal` keeps the original behaviour (everything is omniscient).

Packet layout (little-endian):

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | protocol version |
| 1 | 1 | sender id (0 = ground, drones use `uav_no`) |
| 2 | 2 | sequence number (wraps) |
| 4 | 4 | timestamp, ms |
| 8 | 6 | position x, y, z (int16, decimetres) |
| 14 | 6 | velocity x, y, z (int16, cm/s) |
| 20 | 1 | state (Free / Leader / Slave) |
| 21 | 1 | flags (bit 0 = heartbeat) |
| 22 | 2 | reserved |
| 24 | 8 | HMAC-SHA256 over bytes 0-23, truncated to 64 bits |

Modes:

- `ideal`: no comm layer.
- `fixed_rate`: every drone sends once per heartbeat (1 s).
- `predictive`: a drone sends only when a small predictor (constant velocity or a tiny MLP) of its position, as the peers would estimate it, drifts more than 5 m from the real one, plus the heartbeat. A link is dropped after 3 s of silence.

Each receiver keeps a per-peer trust score from prediction error and RSSI consistency. A peer is quarantined when at least two hearers exist and a strict majority of them score it below the trust threshold.

### Known limitations

- Spoof detection relies on RSSI and degrades when drones are spread more than about 150 m apart.
- A rejected replay is counted but never lowers the claimed sender's trust, so a replayer cannot frame an honest drone.
- Collision avoidance, A* and target scoring still use true positions.
- The replay attacker currently hears without packet loss.

### Benchmark

Run `python scripts/comm_benchmark.py`

## WebSocket Protocol

**Endpoint:** `wss://YOUR_DOMAIN/ws/telemetry`

**Server → Client (every 50 ms):**
```json
{
  "type": "telemetry",
  "drones": [
    { "id": 1, "lat": 41.02, "lng": 28.98, "alt": 50.0,
      "state": "Leader", "vx": 1.2, "vy": 0.3, "vz": 0.0, "speed": 1.24 }
  ],
  "targets": [
    { "id": 1, "lat": 41.01, "lng": 28.97, "state": "Free", "color": "green" }
  ]
}
```

**Client → Server:** Any text frame is treated as a keep-alive ping.

## Key Algorithms

| Component | Algorithm | Complexity |
|-----------|-----------|------------|
| Pathfinding | 3D A* (26-connected grid, Euclidean heuristic) | O(V log V) |
| Collision detection | scipy cKDTree `query_pairs` | O(N log N) |
| Physics integration | Euler (PD thrust + gravity + quadratic drag) | O(1) per drone |
| Target assignment | Greedy nearest-free-goal | O(D·G) |
| Connectivity | DFS on distance-threshold adjacency matrix | O(N²) |

## Running Tests

```bash
pip install pytest pytest-asyncio
pytest tests/ -v
```

Expected: 51 tests, all passing.

## Deployment (VPS)

1. Point DNS A-record to your VPS IP.
2. `sudo apt install certbot python3-certbot-nginx && sudo certbot certonly --nginx -d YOUR_DOMAIN`
3. Edit `nginx/nginx.conf`: replace all `YOUR_DOMAIN_HERE` with your domain.
4. `docker compose up -d --build`
5. Verify: `curl https://YOUR_DOMAIN/api/status`

## Project Structure

```
PythonDrone-master/
├── app/
│   ├── myMath/
│   │   ├── physics.py          # DronePhysics — force models + Euler integrator
│   │   ├── vector.py           # Vector / VectorOperations (numpy-backed)
│   │   └── matrixOperation.py  # Adjacency matrices, connectivity
│   ├── pathfinding.py          # AStarPathfinder — 3D grid A*
│   ├── manage_drones.py        # CollisionDetector, DroneManager (targets, relays)
│   ├── uav.py                  # UAV domain model + physics movement
│   └── goal.py                 # Goal domain model
├── server/
│   ├── main.py                 # FastAPI app — WebSocket, REST routes, lifespan
│   └── simulation_runner.py    # Async simulation loop + metrics
├── map/
│   ├── templates/index.html    # Dashboard UI
│   └── static/js/realtime_map.js  # WebSocket client + rAF animation
├── nginx/nginx.conf            # Reverse proxy + TLS
├── Dockerfile.backend          # Multi-stage Python build
├── docker-compose.yml          # Orchestration
├── .env.example                # Environment variable reference
└── tests/                      # pytest test suite (51 tests)
```
