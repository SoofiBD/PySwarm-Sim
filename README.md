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
pip install fastapi "uvicorn[standard]" numpy scipy pydantic python-multipart jinja2
uvicorn server.main:app --reload
```

Open `http://localhost:8000`.

## Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `MAPS_API_KEY` | No | — | Google Maps JavaScript API key (if omitted, map loads in development mode) |
| `N_DRONES` | No | `6` | Number of drones at startup |
| `N_TARGETS` | No | `8` | Number of targets at startup |
| `SIM_DT` | No | `0.05` | Simulation timestep (seconds) |
| `CTHR` | No | `200.0` | Communication range threshold (metres) |

## REST API

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/status` | WebSocket client count + simulation running state |
| `GET` | `/api/metrics` | Step count, collisions resolved, targets visited |
| `POST` | `/api/control` | `{"action": "start" \| "stop" \| "reset"}` |
| `POST` | `/api/config` | `{"n_drones": N, "n_targets": M}` — reconfigures and restarts |

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

Expected: 45 tests, all passing.

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
│   ├── manage_drones.py        # CollisionDetector, DroneNetwork, DroneManager
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
└── tests/                      # pytest test suite (45 tests)
```
