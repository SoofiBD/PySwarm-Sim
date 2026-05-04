"""
FastAPI WebSocket telemetry server.

Replaces the static-JSON / Flask map server.  The simulation loop runs as
an asyncio background task; each tick it broadcasts a telemetry snapshot to
every connected browser over a single WebSocket endpoint.

Entry point:
    uvicorn server.main:app --host 0.0.0.0 --port 8000 --reload
"""

import asyncio
import json
import math
from contextlib import asynccontextmanager
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from app.myMath.vector import Vector
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground
from server.simulation_runner import SimulationRunner

# ---------------------------------------------------------------------------
# Geographic coordinate conversion
# Simulation space: x ∈ [0, 800], y ∈ [0, 800]  (metres, local Cartesian)
# Reference origin mapped to Istanbul city centre.
# ---------------------------------------------------------------------------
_REF_LAT = 41.015137
_REF_LNG = 28.979530
_ORIGIN_X = 400.0   # simulation units that map to _REF_LAT / _REF_LNG
_ORIGIN_Y = 400.0
_LAT_DEG_PER_M = 1.0 / 111_320.0
_LNG_DEG_PER_M = 1.0 / (111_320.0 * math.cos(math.radians(_REF_LAT)))


def sim_to_geo(x: float, y: float) -> tuple[float, float]:
    lat = _REF_LAT + (y - _ORIGIN_Y) * _LAT_DEG_PER_M
    lng = _REF_LNG + (x - _ORIGIN_X) * _LNG_DEG_PER_M
    return lat, lng


# ---------------------------------------------------------------------------
# Connection manager — tracks active WebSocket clients
# ---------------------------------------------------------------------------
class ConnectionManager:
    def __init__(self) -> None:
        self._active: Set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._active.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self._active.discard(ws)

    async def broadcast(self, payload: dict) -> None:
        text = json.dumps(payload)
        dead: Set[WebSocket] = set()
        for ws in self._active:
            try:
                await ws.send_text(text)
            except Exception:
                dead.add(ws)
        self._active.difference_update(dead)

    @property
    def client_count(self) -> int:
        return len(self._active)


manager = ConnectionManager()
runner: SimulationRunner | None = None


# ---------------------------------------------------------------------------
# Simulation factory — creates domain objects without writing JSON files
# ---------------------------------------------------------------------------
def _create_simulation(
    n_drones: int = 6,
    n_targets: int = 8,
    cthr: float = 200.0,
) -> tuple[list[UAV], list[Goal], Ground]:
    import random
    random.seed(42)

    ground = Ground(pos=Vector(_ORIGIN_X, _ORIGIN_Y, 0.0), cthr=cthr)

    uavs: list[UAV] = []
    for i in range(n_drones):
        x = random.uniform(300, 500)
        y = random.uniform(300, 500)
        uav = UAV(pos=Vector(x, y, 50.0), cthr=cthr, ground=ground)
        uav.setUAVNo(i + 1)
        uav.setIPAddress(f"10.0.0.{i + 2}")
        uavs.append(uav)

    goals: list[Goal] = []
    for j in range(n_targets):
        x = random.uniform(50, 750)
        y = random.uniform(50, 750)
        goal = Goal(pos=Vector(x, y, 0.0))
        goal.setGoalNo(j + 1)
        goals.append(goal)

    return uavs, goals, ground


# ---------------------------------------------------------------------------
# Telemetry serialiser — converts simulation state → broadcast payload
# ---------------------------------------------------------------------------
def _build_telemetry(uavs: list[UAV], goals: list[Goal]) -> dict:
    drones = []
    for uav in uavs:
        lat, lng = sim_to_geo(uav.pos.x, uav.pos.y)
        drones.append({
            "id":    uav.uav_no,
            "lat":   lat,
            "lng":   lng,
            "alt":   round(uav.pos.z, 2),
            "state": uav.state,
            "vx":    round(uav.velocity.x, 3),
            "vy":    round(uav.velocity.y, 3),
            "vz":    round(uav.velocity.z, 3),
            "speed": round(uav.velocity.get_length(), 3),
        })

    targets = []
    for g in goals:
        lat, lng = sim_to_geo(g.pos.x, g.pos.y)
        targets.append({
            "id":    g.goal_no,
            "lat":   lat,
            "lng":   lng,
            "state": g.state,
            "color": g.color,
        })

    return {"type": "telemetry", "drones": drones, "targets": targets}


# ---------------------------------------------------------------------------
# FastAPI application lifecycle
# ---------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    global runner
    uavs, goals, ground = _create_simulation()

    async def _broadcast(uavs_: list, goals_: list) -> None:
        if manager.client_count > 0:
            payload = _build_telemetry(uavs_, goals_)
            await manager.broadcast(payload)

    runner = SimulationRunner(uavs, goals, ground, broadcast_fn=_broadcast)
    task = asyncio.create_task(runner.run_loop(), name="simulation")
    yield
    runner.stop()
    task.cancel()


app = FastAPI(title="DroneSimAPI", version="2.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory="map/static"), name="static")
templates = Jinja2Templates(directory="map/templates")


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.get("/", include_in_schema=False)
async def index(request: Request):
    return templates.TemplateResponse(request, "index.html", {})


@app.get("/api/status")
async def status() -> JSONResponse:
    clients = manager.client_count
    return JSONResponse({"clients_connected": clients, "simulation_running": runner is not None and runner.running})


@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket) -> None:
    """
    Persistent WebSocket channel.
    Server pushes telemetry every simulation tick (~50 ms).
    Client may send any text as a keep-alive ping; it is ignored.
    """
    await manager.connect(websocket)
    try:
        while True:
            # Drain any client pings so the receive buffer does not fill up.
            await asyncio.wait_for(websocket.receive_text(), timeout=30.0)
    except (WebSocketDisconnect, asyncio.TimeoutError):
        pass
    finally:
        manager.disconnect(websocket)
