from PySide6.QtCore import QThread, Signal
from typing import List, Optional
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground
from app.simulation_step import SimulationStep
import time

class SimulationEngine(QThread):
    updated = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.uavs: List[UAV] = []
        self.goals: List[Goal] = []
        self.ground: Optional[Ground] = None
        self.running = False
        self.is_paused = True
        self.cthr = 200.0
        self.interval = 0.05  # 50ms simulation step
        self._sim_step = SimulationStep(cthr=self.cthr, dt=self.interval)

    @property
    def metrics(self) -> dict:
        return self._sim_step.metrics

    def set_data(self, uavs, goals, ground):
        self.uavs = uavs
        self.goals = goals
        self.ground = ground

    def run(self):
        self.running = True
        while self.running:
            if not self.is_paused:
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
