# PythonDrone — FANET Simulation Platform

A high-performance simulation environment for **Heterogeneous Flying Ad-hoc Networks (FANET)**.  
Drones are assigned targets, navigate using **3D A\***, communicate over a mesh, and are rendered live on an interactive **Google Maps** frontend over **WebSocket**.

---

## Architecture Overview

```
Browser (Google Maps)
      │  WebSocket /ws/telemetry  (20 Hz push)
      ▼
  nginx:80
      │  /static/** ──► served from disk (no Python round-trip)
      │  /ws/**     ──► FastAPI backend  (HTTP → WS upgrade)
      │  /**        ──► FastAPI backend
      ▼
FastAPI (server/main.py)
      │  asyncio task
      ▼
SimulationRunner  ──► Physics step (50 ms) ──► Broadcast telemetry
      │
      ├── DroneManager   — target assignment, relay logic
      ├── CollisionDetector — KD-tree O(N log N) avoidance
      ├── DroneNetwork   — comm-graph target sharing
      └── UAV.move()     — PD-controller + gravity + drag (Newton)
```

---

## Key Features

| Feature | Detail |
|---------|--------|
| **Real-time telemetry** | WebSocket push at 20 Hz; 60 fps smooth interpolation in the browser |
| **3D A\* pathfinding** | 26-connected grid, Euclidean heuristic, dynamic obstacle support |
| **Classical mechanics** | PD-controller thrust, quadratic drag, gravity — Euler integration |
| **Optimized collision detection** | `scipy.cKDTree.query_pairs` — O(N log N) vs old O(N²) matrix scan |
| **Drone-to-drone comms** | Mesh graph built each tick; targets shared within comm range |
| **Relay assignment** | Automatic relay when network splits into disconnected components |
| **Docker deployment** | Multi-stage build, nginx reverse proxy, health-checked compose stack |

---

## Technology Stack

| Layer | Library / Tool |
|-------|---------------|
| WebSocket server | **FastAPI** + **uvicorn** |
| Simulation math | **NumPy**, **SciPy** (cKDTree) |
| Domain models | **Pydantic v2** |
| Map frontend | **Google Maps JS API** + native WebSocket |
| Reverse proxy | **nginx 1.25** |
| Containerisation | **Docker** + **docker-compose** |
| Desktop UI (legacy) | **PySide6** |
| Testing | **pytest**, **pytest-asyncio**, **httpx** |
| Language | **Python 3.11+** |

---

## Project Structure

```
PythonDrone-master/
├── server/
│   ├── main.py              # FastAPI app — WebSocket, lifespan, coord mapping
│   └── simulation_runner.py # Async simulation loop (replaces QThread engine)
│
├── app/
│   ├── engine.py            # Legacy PySide6 QThread engine (desktop path)
│   ├── uav.py               # UAV model — physics-based move(), velocity, mass
│   ├── goal.py              # Target model
│   ├── ground.py            # Base station model
│   ├── node.py              # Shared pydantic base
│   ├── manage_drones.py     # DroneManager, CollisionDetector (KD-tree), DroneNetwork
│   ├── pathfinding.py       # 3D A* pathfinder
│   ├── generateUAV.py       # UAV factory (legacy file-write path)
│   ├── generateGoal.py      # Goal factory (legacy file-write path)
│   └── myMath/
│       ├── vector.py        # Vector & VectorOperations
│       ├── matrix.py        # Matrix class
│       ├── matrixOperation.py # Adjacency matrix, connected components
│       ├── physics.py       # Force models: thrust, gravity, drag
│       └── trigonometry.py
│
├── map/
│   ├── templates/index.html # WebSocket-driven map UI (dark theme)
│   └── static/
│       ├── js/realtime_map.js  # WS client, smooth marker interpolation
│       └── css/
│
├── nginx/
│   └── nginx.conf           # Reverse proxy + static file server
│
├── tests/
│   ├── test_collision.py
│   ├── test_matrix_operation.py
│   └── test_vector.py
│
├── Dockerfile.backend       # Multi-stage Python image
├── docker-compose.yml       # backend + nginx, internal network
├── pyproject.toml
└── run.py                   # Legacy desktop UI entry point
```

---

## Quick Start — Local Development

```bash
git clone https://github.com/SoofiBD/PythonDrone-master.git
cd PythonDrone-master

python -m venv venv
source venv/bin/activate      # Windows: venv\Scripts\activate

pip install -e ".[dev]"       # installs all dependencies from pyproject.toml

uvicorn server.main:app --host 0.0.0.0 --port 8000 --reload
```

Open `http://localhost:8000` — the map loads and telemetry begins streaming immediately.

---

## Docker Deployment (VPS)

```bash
# Build and start both services
docker compose up --build -d

# Tail logs
docker compose logs -f

# Stop
docker compose down
```

nginx listens on port 80.  Static assets are served from disk with 1-hour cache headers; the WebSocket and API are proxied to the FastAPI backend on the internal network.

**HTTPS (Let's Encrypt):**
```bash
certbot certonly --nginx -d your-domain.com
# Then uncomment the TLS lines in nginx/nginx.conf and docker-compose.yml
```

---

## Physics Model

Each drone integrates Newton's second law every 50 ms:

```
F_net = F_thrust  +  F_gravity  +  F_drag

F_thrust = m·(kp·(x_goal − x) − kd·v)  +  m·g·ẑ    [PD controller, clamped to F_max]
F_gravity = [0, 0, −m·g]                              [g = 9.81 m/s²]
F_drag    = −½·ρ·Cd·A·|v|²·v̂                         [quadratic Newtonian drag]

a = F_net / m
v(t+dt) = v(t) + a·dt                                 [Euler integration, dt = 0.05 s]
x(t+dt) = x(t) + v(t+dt)·dt
```

Default parameters (350-class quadcopter):

| Parameter | Value |
|-----------|-------|
| mass | 1.5 kg |
| max_thrust | 30 N (4 × 7.5 N motors) |
| air density ρ | 1.225 kg/m³ |
| drag coeff Cd | 0.47 |
| reference area A | 0.04 m² |
| kp (proportional gain) | 3.0 |
| kd (derivative gain) | 2.0 |

---

## Pathfinding — 3D A\*

`app/pathfinding.py` implements A\* over a **26-connected 3D grid** (face + edge + corner neighbours).

- **Heuristic**: Euclidean distance — admissible, guarantees optimal paths.
- **Complexity**: O(V log V), V = explored nodes.
- **Obstacles**: Dynamic positions (other drones) passed per call; blocked nodes skipped during expansion.

```python
from app.pathfinding import AStarPathfinder
from app.myMath.vector import Vector

pf = AStarPathfinder(grid_resolution=40.0)
path = pf.find_path(
    start=Vector(0, 0, 50),
    goal=Vector(400, 400, 50),
    obstacles=[Vector(200, 200, 50)],   # other drone positions
)
# path → [Vector(0,0,50), Vector(40,40,50), ..., Vector(400,400,50)]
```

---

## Collision Detection

`CollisionDetector` uses **`scipy.spatial.cKDTree.query_pairs(r)`** to find all pairs within the minimum safe distance in one call:

| N drones | Old O(N²) | New O(N log N) | Speedup |
|----------|-----------|----------------|---------|
| 20 | ~0.04 ms | ~0.01 ms | 4× |
| 100 | ~1.0 ms | ~0.08 ms | 12× |
| 500 | ~25 ms | ~0.5 ms | 50× |

---

## Configuration

**Simulation parameters** (`server/simulation_runner.py`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dt` | 0.05 s | Physics integration timestep |
| `cthr` | 200.0 | Connectivity threshold (simulation units) |

**Collision / comms** (`app/manage_drones.py`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CollisionDetector.MIN_SAFE_DISTANCE` | 5.0 | Avoidance trigger distance |
| `DroneNetwork.COMM_RANGE` | 150.0 | Drone-to-drone comm range |

**Physics** (`app/uav.py`):

| Field | Default | Description |
|-------|---------|-------------|
| `mass` | 1.5 kg | Drone mass |
| `max_thrust` | 30.0 N | Motor thrust ceiling |

---

## Running Tests

```bash
pytest tests/ -v
```

---

## Logging

Events are written to `logs/simulation.log` and stdout:

| Tag | Meaning |
|-----|---------|
| `[CONNECTIVITY]` | Number of network components this tick |
| `[RELAY]` | Relay assignment triggered |
| `[NETWORK]` | Active drone-to-drone links |
| `[COMM]` | Target info shared between drones |

---

*Developed as part of modernizing FANET research tooling.*
