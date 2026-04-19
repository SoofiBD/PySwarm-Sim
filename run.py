import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Qt
from View import View
from app.engine import SimulationEngine
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground
from app.myMath.vector import Vector
from app.myMath.matrixOperation import MatrixOperation
from app.myMath.matrix import Matrix
from app.generateUAV import GenerateUAV
from app.generateGoal import GenerateGoal

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.view = View()
        self.setCentralWidget(self.view)
        
        # State
        self.uavs = []
        self.goals = []
        self.ground = None
        
        # Engine
        self.engine = SimulationEngine()
        self.engine.updated.connect(self.on_sim_update)
        self.engine.start() # Start the thread
        
        # Connect View Signals
        self.view.start_requested.connect(self.engine.resume)
        self.view.stop_requested.connect(self.engine.pause)
        self.view.reset_requested.connect(self.reset_simulation)
        self.view.uav_generate_requested.connect(self.generate_uavs)
        self.view.goal_generate_requested.connect(self.generate_goals)
        self.view.ground_generate_requested.connect(self.set_ground)

    def on_sim_update(self):
        """Update UI based on simulation state."""
        # 1. Update Adjacency Matrix Display
        if self.uavs:
            if self.ground:
                adj = MatrixOperation.UAVtoGround_AdjMatrix(self.uavs, self.ground)
            else:
                adj = MatrixOperation.UAVtoUAV_AdjMatrix(self.uavs)
            self.view.adj_display.setText(Matrix.MatrixAsString(adj))
        
        # 2. Redraw Plot
        self.update_plot()

    def update_plot(self):
        ax = self.view.sc.axes
        ax.cla()
        
        # Set limits and style
        ax.set_xlim([0, 800])
        ax.set_ylim([0, 800])
        ax.set_zlim([0, 800])
        ax.set_facecolor('#1e1e1e')
        
        # Draw Ground
        if self.ground:
            ax.plot(self.ground.pos.x, self.ground.pos.y, self.ground.pos.z,
                    marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
            
        # Draw UAVs
        for uav in self.uavs:
            ax.plot(uav.pos.x, uav.pos.y, uav.pos.z,
                    marker="x", markersize=8, markeredgecolor="red")
            
        # Draw Goals
        for goal in self.goals:
            ax.plot(goal.pos.x, goal.pos.y, goal.pos.z,
                    marker="o", markersize=8, markeredgecolor="green", markerfacecolor=goal.color)
            
        # Draw Connections
        if len(self.uavs) > 1:
            for i in range(len(self.uavs)):
                for j in range(i):
                    dist = MatrixOperation.distance(self.uavs[i].pos, self.uavs[j].pos)
                    if dist <= self.engine.cthr:
                        ax.plot([self.uavs[i].pos.x, self.uavs[j].pos.x],
                                [self.uavs[i].pos.y, self.uavs[j].pos.y],
                                [self.uavs[i].pos.z, self.uavs[j].pos.z], color="blue", alpha=0.5)
            
            if self.ground:
                for uav in self.uavs:
                    if MatrixOperation.distance(uav.pos, self.ground.pos) <= self.engine.cthr:
                        ax.plot([uav.pos.x, self.ground.pos.x],
                                [uav.pos.y, self.ground.pos.y],
                                [uav.pos.z, self.ground.pos.z], color="blue", alpha=0.5)

        self.view.sc.draw()

    def generate_uavs(self, count):
        self.uavs = GenerateUAV.Run(count=count, loctype=1, cthr=self.engine.cthr, UAVs=self.uavs, ground=self.ground)
        self.engine.set_data(self.uavs, self.goals, self.ground)
        self.update_plot()

    def generate_goals(self, count):
        self.goals = GenerateGoal.Run(count=count, Goals=self.goals)
        self.engine.set_data(self.uavs, self.goals, self.ground)
        self.update_plot()

    def set_ground(self, vector):
        self.ground = Ground(pos=vector, cthr=self.engine.cthr)
        self.engine.set_data(self.uavs, self.goals, self.ground)
        self.update_plot()

    def reset_simulation(self):
        self.engine.pause()
        self.uavs = []
        self.goals = []
        self.ground = None
        self.engine.set_data(self.uavs, self.goals, self.ground)
        self.update_plot()
        self.view.adj_display.setText("Matrix cleared.")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
