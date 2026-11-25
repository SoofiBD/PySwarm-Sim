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
| `SWARM_KEY` | No | dev key | Hex HMAC key for drone packets, at least 16 bytes (32 hex characters). If unset or empty a built-in development key is used and a warning is logged |
| `COMM_MODE` | No | `ideal` | Startup comm mode: `ideal`, `fixed_rate` or `predictive`. An invalid value falls back to `ideal`. `POST /api/config` overrides it on restart |

## REST API

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/status` | WebSocket client count + simulation running state |
| `GET` | `/api/metrics` | Step count, collisions resolved, targets visited |
| `POST` | `/api/control` | `{"action": "start" \| "stop" \| "reset"}` |
| `POST` | `/api/config` | `{"n_drones": N, "n_targets": M, "comm_mode": "ideal" \| "fixed_rate" \| "predictive"}` — reconfigures and restarts. `comm_mode` is optional (default `ideal`) |
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
- `fixed_rate`: every drone sends a packet every simulation tick (20 Hz at the default 50 ms step).
- `predictive`: a drone sends only when a small predictor (constant velocity or a tiny MLP) of its position, as the peers would estimate it, drifts more than 5 m from the real one, plus the heartbeat. A link is dropped after 3 s of silence. Relay assignment waits for the first 3 s so the swarm isn't split up before anyone has heard anyone.

Each receiver keeps a per-peer trust score from prediction error and RSSI consistency. A peer is quarantined when at least two hearers exist and a strict majority of them score it below the trust threshold. Quarantined drones get no new targets and are never used as relays.

### Known limitations

- Spoof detection relies on RSSI and degrades when drones are spread more than about 150 m apart.
- A replay that is rejected (old sequence number or timestamp) is only counted and never lowers the claimed sender's trust. A replay with a short delay (for example `inject_replay(pos, delay_s=0.05)`) can still hurt an honest drone: a receiver that missed the original packet accepts the copy as fresh, and because the RSSI comes from the attacker's position instead of the sender's, the RSSI check fails and the sender's trust drops.
- Collision avoidance, A* and target scoring still use true positions.
- With a comm mode on, relay drones are reassigned more often and can pass each other closer than the 5 m safety distance (down to about 2 m in a 120 s run on the default layout). Avoidance only adds a small velocity nudge; a real airframe needs onboard sensing for this. `ideal` mode keeps at least 8 m.
- The replay attacker currently hears without packet loss.

### Benchmark

```bash
python -m scripts.comm_benchmark --seconds 120 --seed 100
```

`tight` is the compact drifting cluster used in `tests/test_comm_network.py`; `default` is the server's spawn layout flying real missions through `SimulationStep`. Each row is one clean run plus one run where drone 3 is GPS-spoofed at 5 s.

| layout | mode / predictor | packets/s | bytes/s | belief error (m) | spoof time to detect | false quarantine (clean / spoof) |
|---|---|---:|---:|---:|---|---|
| tight | fixed_rate / cv | 140.0 | 4480 | 0.03 | 16.5 s | 0.000 / 0.000 |
| tight | predictive / cv | 7.0 | 224 | 0.04 | 35.0 s | 0.000 / 0.000 |
| tight | predictive / mlp | 7.3 | 234 | 2.74 | 33.9 s | 0.000 / 0.000 |
| default | fixed_rate / cv | 140.0 | 4480 | 0.91 | 33.1 s | 0.000 / 0.000 |
| default | predictive / cv | 7.7 | 246 | 3.06 | not detected in 115 s | 0.000 / 0.000 |
| default | predictive / mlp | 7.7 | 245 | 4.43 | not detected in 115 s | 0.000 / 0.000 |

Predictive sending cuts traffic by about 18-20x while keeping the belief error under the 5 m send threshold. Spoof detection is slower with fewer packets: 35 s instead of 16.5 s in the tight cluster. On the spread-out default layout only `fixed_rate` caught the spoof (33 s); with predictive sending it was not caught within the run, and the result moves a lot with small changes to the scenario, so treat RSSI-based detection on spread swarms as unreliable. No honest drone was quarantined in any run. The MLP did not beat constant velocity on belief error in either layout, so `CommNetwork` keeps constant velocity as its default predictor.

The MLP weights (`app/comm/predictor_weights.npz`) come from `python -m scripts.train_predictor --runs 20 --seed 0` (about 10 s); `python -m scripts.export_predictor_c` writes them as an int8 C header to `build/predictor_weights.h`.

### Roadmap

The comm layer works, but two things hold it back: neighbours predict each other without knowing where anyone is heading, and RSSI is too noisy to verify positions once the swarm spreads out. The next steps, in order:

1. **Shared intent.** Add the current waypoint to the packet so every drone can run the same small motion model for its neighbours. Retrain the MLP on the residual that model misses (wind, battery sag, manoeuvres) instead of on blind constant-velocity error. Goal: fewer packets and a predictor that actually beats the baseline.
2. **Learned trust.** Replace the hand-tuned trust thresholds with a small classifier (a few hundred parameters, int8 on the ESP32) trained in simulation on spoofing, sensor-fault and replay scenarios. Inputs: prediction residual, RSSI consistency, and whether several witnesses of the same peer agree.
3. **FTM ranging.** Model Wi-Fi Fine Timing Measurement (802.11mc, supported by the ESP32-S3) as a 1-2 m range source next to RSSI, and check a peer's claimed position against ranges from several witnesses. This should make spoof detection work on spread-out swarms.
4. **Risk-aware bandwidth.** Scale the send threshold with collision risk: drones close to each other report more often, distant ones less. This also targets the close relay passes listed above.
5. **Hardware in the loop.** Run 2-3 ESP32-S3 boards with ESP-NOW against the simulator: the sim feeds each board its position, the boards exchange real packets over the air, and the sim reads back their beliefs and trust scores.

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

Expected: 115 tests, all passing.

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
└── tests/                      # pytest test suite (115 tests)
```
