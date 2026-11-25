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
import logging
import math
import os
import secrets
from contextlib import asynccontextmanager
from typing import Set

from fastapi import Depends, FastAPI, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel as PydanticBase, Field
from typing import Literal

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
# Request models
# ---------------------------------------------------------------------------
class ControlRequest(PydanticBase):
    action: Literal["start", "stop", "reset"]

class ConfigRequest(PydanticBase):
    n_drones: int = Field(default=6, ge=1, le=100)
    n_targets: int = Field(default=8, ge=1, le=200)
    comm_mode: Literal["ideal", "fixed_rate", "predictive"] = "ideal"

class FaultRequest(PydanticBase):
    kind: Literal["gps_spoof", "replay"]
    drone_id: int | None = None


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
        if not self._active:
            return
        text = json.dumps(payload)
        dead: Set[WebSocket] = set()

        async def _send(ws: WebSocket) -> WebSocket | None:
            try:
                await asyncio.wait_for(ws.send_text(text), timeout=1.0)
                return None
            except Exception:
                return ws

        results = await asyncio.gather(*[_send(ws) for ws in list(self._active)], return_exceptions=False)
        for ws in results:
            if ws is not None:
                dead.add(ws)
        self._active.difference_update(dead)

    @property
    def client_count(self) -> int:
        return len(self._active)


_COMM_MODES = ("ideal", "fixed_rate", "predictive")


def _env_comm_mode() -> str:
    mode = os.getenv("COMM_MODE", "ideal")
    if mode not in _COMM_MODES:
        logging.getLogger("DroneSimulation").warning("invalid COMM_MODE %r, using ideal", mode)
        return "ideal"
    return mode


manager = ConnectionManager()
runner: SimulationRunner | None = None
_runner_task: asyncio.Task | None = None
# Serialises start/stop/reset/config so two requests can't spawn two loops.
_control_lock = asyncio.Lock()


def _require_admin(x_admin_token: str | None = Header(default=None)) -> None:
    # Unset ADMIN_TOKEN = local dev, endpoints open. docker-compose requires it.
    expected = os.getenv("ADMIN_TOKEN", "")
    if expected and not secrets.compare_digest(x_admin_token or "", expected):
        raise HTTPException(status_code=401, detail="invalid admin token")


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


def _make_broadcast_fn():
    async def _broadcast(uavs_: list, goals_: list) -> None:
        if manager.client_count > 0:
            payload = _build_telemetry(uavs_, goals_, runner.comm if runner else None)
            await manager.broadcast(payload)
    return _broadcast


# ---------------------------------------------------------------------------
# Telemetry serialiser — converts simulation state → broadcast payload
# ---------------------------------------------------------------------------
def _build_telemetry(uavs: list[UAV], goals: list[Goal], comm=None) -> dict:
    drones = []
    quarantined = comm.quarantined_ids() if comm else set()
    suspect = comm.suspect_ids() if comm else set()
    for i, uav in enumerate(uavs):
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
        if comm:
            nid = comm.node_id(uav, i)
            drones[-1].update(trust=round(comm.mean_trust(nid), 3),
                              quarantined=nid in quarantined,
                              suspect=nid in suspect)

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


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# FastAPI application lifecycle
# ---------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    global runner, _runner_task
    n_drones  = int(os.getenv("N_DRONES", "6"))
    n_targets = int(os.getenv("N_TARGETS", "8"))
    cthr      = _clamp(float(os.getenv("CTHR", "200.0")), 10.0, 1000.0)
    dt        = _clamp(float(os.getenv("SIM_DT", "0.05")), 0.01, 0.2)
    uavs, goals, ground = _create_simulation(n_drones=n_drones,
                                              n_targets=n_targets,
                                              cthr=cthr)
    runner = SimulationRunner(uavs, goals, ground,
                              broadcast_fn=_make_broadcast_fn(), dt=dt, cthr=cthr,
                              comm_mode=_env_comm_mode())
    _runner_task = asyncio.create_task(runner.run_loop(), name="simulation")
    yield
    await _stop_runner()


app = FastAPI(title="DroneSimAPI", version="2.0.0", lifespan=lifespan)

_allowed_origins = [o.strip() for o in os.getenv("ALLOWED_ORIGINS", "*").split(",") if o.strip()]
app.add_middleware(
    CORSMiddleware,
    allow_origins=_allowed_origins,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory="map/static"), name="static")
templates = Jinja2Templates(directory="map/templates")


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.get("/", include_in_schema=False)
async def index(request: Request):
    return templates.TemplateResponse(request, "index.html", {
        "maps_api_key": os.getenv("MAPS_API_KEY", ""),
    })


@app.get("/api/status")
async def status() -> JSONResponse:
    clients = manager.client_count
    return JSONResponse({"clients_connected": clients, "simulation_running": runner is not None and runner.running})


@app.get("/api/metrics")
async def metrics() -> JSONResponse:
    if runner is None:
        return JSONResponse({"error": "simulation not running"}, status_code=503)
    return JSONResponse({
        **({"comm": runner.metrics["comm"]} if "comm" in runner.metrics else {}),
        "steps": runner.metrics["steps"],
        "collisions_resolved": runner.metrics["collisions_resolved"],
        "targets_visited": runner.metrics["targets_visited"],
        "drones": len(runner.uavs),
        "targets": len(runner.goals),
    })


async def _stop_runner() -> None:
    # Cancel (not just flag) so a sleeping loop can't wake up after a quick restart.
    global _runner_task
    if runner:
        runner.stop()
    if _runner_task:
        _runner_task.cancel()
        try:
            await _runner_task
        except asyncio.CancelledError:
            pass
        _runner_task = None


async def _restart_runner(n_drones: int = 6, n_targets: int = 8, cthr: float = 200.0, dt: float = 0.05,
                          comm_mode: str | None = None) -> None:
    global runner, _runner_task
    if comm_mode is None:
        comm_mode = runner.comm_mode if runner else _env_comm_mode()
    await _stop_runner()
    cthr = _clamp(cthr, 10.0, 1000.0)
    dt = _clamp(dt, 0.01, 0.2)
    uavs, goals, ground = _create_simulation(n_drones=n_drones, n_targets=n_targets, cthr=cthr)
    runner = SimulationRunner(uavs, goals, ground, broadcast_fn=_make_broadcast_fn(), dt=dt, cthr=cthr,
                              comm_mode=comm_mode)
    _runner_task = asyncio.create_task(runner.run_loop(), name="simulation")


@app.post("/api/control", dependencies=[Depends(_require_admin)])
async def control(req: ControlRequest) -> JSONResponse:
    global _runner_task
    async with _control_lock:
        if req.action == "stop":
            await _stop_runner()
            return JSONResponse({"status": "stopped"})
        elif req.action == "start":
            if runner and _runner_task is None:
                _runner_task = asyncio.create_task(runner.run_loop(), name="simulation")
            return JSONResponse({"status": "started"})
        elif req.action == "reset":
            await _restart_runner()
            return JSONResponse({"status": "reset"})
    return JSONResponse({"error": f"unknown action: {req.action}"}, status_code=400)


@app.post("/api/config", dependencies=[Depends(_require_admin)])
async def configure(req: ConfigRequest) -> JSONResponse:
    cthr = _clamp(float(os.getenv("CTHR", "200.0")), 10.0, 1000.0)
    dt = _clamp(float(os.getenv("SIM_DT", "0.05")), 0.01, 0.2)
    async with _control_lock:
        await _restart_runner(n_drones=req.n_drones, n_targets=req.n_targets, cthr=cthr, dt=dt,
                              comm_mode=req.comm_mode)
    return JSONResponse({"status": "reconfigured",
                         "n_drones": req.n_drones,
                         "n_targets": req.n_targets})


@app.post("/api/fault", dependencies=[Depends(_require_admin)])
async def fault(req: FaultRequest) -> JSONResponse:
    async with _control_lock:
        comm = runner.comm if runner else None
        if comm is None:
            return JSONResponse({"error": "comm mode is ideal"}, status_code=409)
        ids = [comm.node_id(u, i) for i, u in enumerate(runner.uavs)]
        if req.drone_id is not None and req.drone_id not in ids:
            return JSONResponse({"error": "unknown drone_id"}, status_code=404)
        if req.kind == "gps_spoof":
            comm.inject_gps_spoof(req.drone_id if req.drone_id is not None else ids[0])
        else:
            n = len(runner.uavs)
            comm.inject_replay(tuple(sum(u.pos._data[k] for u in runner.uavs) / n for k in range(3)))
    return JSONResponse({"status": "injected"})


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
