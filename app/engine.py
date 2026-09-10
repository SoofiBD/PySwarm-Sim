import threading
import time
from typing import List, Optional

from PySide6.QtCore import QThread, Signal

from app.goal import Goal
from app.ground import Ground
from app.simulation_step import SimulationStep
from app.uav import UAV

class SimulationEngine(QThread):
    updated = Signal()

    def __init__(self, parent=None, cthr: float = 200.0, interval: float = 0.05):
        super().__init__(parent)
        self.uavs: List[UAV] = []
        self.goals: List[Goal] = []
        self.ground: Optional[Ground] = None
        self.running = False
        self.is_paused = True
        self.cthr = cthr
        self.interval = max(0.01, min(0.2, interval))
        self._lock = threading.Lock()
        self._sim_step = SimulationStep(cthr=self.cthr, dt=self.interval)

    @property
    def metrics(self) -> dict:
        return self._sim_step.metrics

    def set_data(self, uavs, goals, ground):
        with self._lock:
            self.uavs = uavs
            self.goals = goals
            self.ground = ground

    def update_config(self, cthr: float | None = None, dt: float | None = None) -> None:
        if cthr is not None:
            self.cthr = max(10.0, min(1000.0, cthr))
            self._sim_step.cthr = self.cthr
        if dt is not None:
            self.interval = max(0.01, min(0.2, dt))
            self._sim_step.dt = self.interval

    def run(self):
        self.running = True
        while self.running:
            if not self.is_paused:
                with self._lock:
                    uavs, goals, ground = self.uavs, self.goals, self.ground
                # tick outside lock where possible? but uavs list mutated, so copy ref is okay;
                # tick mutates UAV objects — keep lock for full tick to avoid set_data race
                with self._lock:
                    self._sim_step.tick(self.uavs, self.goals, self.ground)
                self.updated.emit()
            time.sleep(self.interval)

    def stop(self):
        self.running = False
        self.wait()

    def pause(self):
        self.is_paused = True

    def resume(self):
        self.is_paused = False
