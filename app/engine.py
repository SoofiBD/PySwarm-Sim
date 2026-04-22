from PySide6.QtCore import QThread, Signal, QTimer
from typing import List, Optional
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground
from app.myMath.matrixOperation import MatrixOperation
from app.manage_drones import DroneManager, CollisionDetector, DroneNetwork
from app.logger import log_simulation_event, log_error
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

    def set_data(self, uavs, goals, ground):
        self.uavs = uavs
        self.goals = goals
        self.ground = ground

    def run(self):
        self.running = True
        while self.running:
            if not self.is_paused:
                self.step()
                self.updated.emit()
            time.sleep(self.interval)

    def step(self):
        """Single simulation step logic."""
        if not self.uavs:
            return

        # 1. Update Adjacency Matrix
        if self.ground:
            adj = MatrixOperation.UAVtoGround_AdjMatrix(self.uavs, self.ground)
        else:
            adj = MatrixOperation.UAVtoUAV_AdjMatrix(self.uavs)

        # 2. Reset states for reallocation if needed
        # (Follows the logic in the original run.py update loop)
        DroneManager.ResetUAVs(self.uavs)
        DroneManager.ResetGoals(self.goals)

        # 3. Drone Communication - Share target info
        comm_graph = DroneNetwork.build_communication_graph(self.uavs)
        connected_pairs = sum(len(v) for v in comm_graph.values()) // 2
        log_simulation_event("NETWORK", f"{connected_pairs} drone-to-drone links")

        DroneNetwork.share_targets(self.uavs)

        for uav in self.uavs:
            if uav.getTarget() and len(uav.known_targets) > 0:
                log_simulation_event("COMM", f"UAV {uav.uav_no} shares target with {len(uav.known_targets)} drones")

        # 4. Target Assignment
        i = 0
        free_uavs = DroneManager.GetFreeUAVNumber(self.uavs)
        free_goals = DroneManager.GetFreeGoalNumber(self.goals)

        while free_uavs > 0 and free_goals > 0:
            if self.uavs[i].getState() == "Free":
                target = DroneManager.AssignTarget(self.goals, self.uavs, self.uavs[i])
                if target:
                    self.uavs[i].setTarget(target)
                free_uavs -= 1
                free_goals -= 1
            i = (i + 1) % len(self.uavs)
            if i == 0: break # Safety exit

        # 5. Connectivity / Relay Management
        # Find connection components
        components = MatrixOperation.find_conn_comp(adj, self.cthr)
        log_simulation_event("CONNECTIVITY", f"{len(components)} components detected")

        if len(components) != 1:
            # Assign Relays to try and bridge the gap
            # Note: This logic is still being refined in manage_drones.py
            RelayUAVs = DroneManager.AssignRelay(self.uavs, adj, self.ground, self.cthr)
            log_simulation_event("RELAY", f"{len(RelayUAVs)} drones assigned as relays")

            for uav in self.uavs:
                if uav.getState() == "Slave":
                    uav.followLeader()

        # 6. Collision Avoidance
        CollisionDetector.apply_avoidance(self.uavs)

        # 6. Movement
        for uav in self.uavs:
            target = uav.getTarget()
            if target and uav.getState() == "Leader":
                uav.move(target)

    def stop(self):
        self.running = False
        self.wait()

    def pause(self):
        self.is_paused = True

    def resume(self):
        self.is_paused = False
