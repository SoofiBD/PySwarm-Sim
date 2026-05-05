from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QLineEdit, QGroupBox, QGridLayout, QSizePolicy, 
                             QMessageBox, QFrame)
from PySide6.QtCore import Signal, Qt, QTimer
from PySide6.QtGui import QIcon, QPalette, QColor, QFont
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from app.myMath.vector import Vector
from app.Validators import XYZValidator, numValidator

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=5, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor='#1e1e1e')
        self.axes = self.fig.add_subplot(111, projection='3d')
        self.axes.set_facecolor('#1e1e1e')
        
        # Style the axes
        for axis in [self.axes.xaxis, self.axes.yaxis, self.axes.zaxis]:
            axis.set_pane_color((0.15, 0.15, 0.15, 1.0))
            axis.label.set_color('white')
            axis.set_tick_params(colors='white')
            
        self.axes.set_xlim([0, 800])
        self.axes.set_ylim([0, 800])
        self.axes.set_zlim([0, 800])
        super().__init__(self.fig)

class View(QWidget):
    # Signals
    goal_add_requested = Signal(Vector)
    uav_add_requested = Signal(Vector)
    ground_generate_requested = Signal(Vector)
    goal_generate_requested = Signal(int)
    uav_generate_requested = Signal(int)
    start_requested = Signal()
    stop_requested = Signal()
    reset_requested = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Heterojen Uçan Tasarsız Ağlar Test Ortamı")
        self.setMinimumSize(1200, 800)
        self._init_ui()
        self._set_style()

    def _init_ui(self):
        main_layout = QHBoxLayout(self)
        
        # Left Side: Visualization
        viz_container = QWidget()
        viz_layout = QVBoxLayout(viz_container)
        
        self.sc = MplCanvas(self, width=8, height=8)
        self.toolbar = NavigationToolbar(self.sc, self)
        
        viz_layout.addWidget(self.toolbar)
        viz_layout.addWidget(self.sc)
        main_layout.addWidget(viz_container, stretch=3)

        # Right Side: Controls and Stats
        right_panel = QWidget()
        right_panel.setFixedWidth(350)
        right_layout = QVBoxLayout(right_panel)
        main_layout.addWidget(right_panel, stretch=1)

        # 1. Adjacency Matrix
        adj_group = QGroupBox("Adjacency Matrix")
        adj_layout = QVBoxLayout(adj_group)
        self.adj_display = QLabel("Matrix will appear here...")
        self.adj_display.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.adj_display.setFont(QFont("Courier New", 8))
        self.adj_display.setStyleSheet("color: #00ff00; background-color: #000; padding: 5px;")
        self.adj_display.setWordWrap(True)
        adj_layout.addWidget(self.adj_display)
        right_layout.addWidget(adj_group, stretch=1)

        # 2. Settings Group
        settings_group = QGroupBox("Simulation Settings")
        settings_layout = QGridLayout(settings_group)
        
        # Inputs
        self.uav_count_input = QLineEdit()
        self.uav_count_input.setPlaceholderText("Count")
        self.uav_count_input.setValidator(numValidator())
        
        self.uav_gen_btn = QPushButton("Gen UAVs")
        self.uav_gen_btn.clicked.connect(self._on_uav_gen)
        
        settings_layout.addWidget(QLabel("UAVs:"), 0, 0)
        settings_layout.addWidget(self.uav_count_input, 0, 1)
        settings_layout.addWidget(self.uav_gen_btn, 0, 2)

        self.goal_count_input = QLineEdit()
        self.goal_count_input.setPlaceholderText("Count")
        self.goal_count_input.setValidator(numValidator())
        self.goal_gen_btn = QPushButton("Gen Goals")
        self.goal_gen_btn.clicked.connect(self._on_goal_gen)
        
        settings_layout.addWidget(QLabel("Goals:"), 1, 0)
        settings_layout.addWidget(self.goal_count_input, 1, 1)
        settings_layout.addWidget(self.goal_gen_btn, 1, 2)

        # Ground Position
        self.gx = QLineEdit(); self.gy = QLineEdit(); self.gz = QLineEdit()
        for i in [self.gx, self.gy, self.gz]: 
            i.setPlaceholderText("0")
            i.setValidator(XYZValidator())
            i.setFixedWidth(40)
            
        ground_btn = QPushButton("Set Ground")
        ground_btn.clicked.connect(self._on_ground_set)
        
        settings_layout.addWidget(QLabel("Ground (X,Y,Z):"), 2, 0)
        settings_layout.addWidget(self.gx, 2, 1)
        settings_layout.addWidget(self.gy, 2, 2)
        settings_layout.addWidget(self.gz, 2, 3)
        settings_layout.addWidget(ground_btn, 2, 4 if 2==2 else 2) # Adjust grid if needed

        right_layout.addWidget(settings_group)

        # 3. Control Buttons
        ctrl_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.start_btn.setObjectName("startBtn")
        self.start_btn.clicked.connect(self._on_start_toggle)
        
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_requested.emit)
        
        ctrl_layout.addWidget(self.start_btn)
        ctrl_layout.addWidget(self.reset_btn)
        right_layout.addLayout(ctrl_layout)

    def _set_style(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #2d2d2d;
                color: #e0e0e0;
                font-family: 'Segoe UI', Arial;
            }
            QGroupBox {
                border: 2px solid #3d3d3d;
                border-radius: 8px;
                margin-top: 1ex;
                font-weight: bold;
            }
            QPushButton {
                background-color: #4a4a4a;
                border: none;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5a5a5a;
            }
            QPushButton#startBtn {
                background-color: #2e7d32;
            }
            QPushButton#startBtn:checked {
                background-color: #c62828;
            }
            QLineEdit {
                background-color: #3d3d3d;
                border: 1px solid #5a5a5a;
                padding: 4px;
                border-radius: 2px;
            }
        """)

    def _on_start_toggle(self):
        if self.start_btn.text() == "Start":
            self.start_btn.setText("Stop")
            self.start_btn.setStyleSheet("background-color: #c62828;")
            self.start_requested.emit()
        else:
            self.start_btn.setText("Start")
            self.start_btn.setStyleSheet("background-color: #2e7d32;")
            self.stop_requested.emit()

    def _on_uav_gen(self):
        try:
            count = int(self.uav_count_input.text())
            self.uav_generate_requested.emit(count)
        except ValueError:
            QMessageBox.warning(self, "Hata", "Lütfen geçerli bir sayı girin.")

    def _on_goal_gen(self):
        try:
            count = int(self.goal_count_input.text())
            self.goal_generate_requested.emit(count)
        except ValueError:
            QMessageBox.warning(self, "Hata", "Lütfen geçerli bir sayı girin.")

    def _on_ground_set(self):
        try:
            v = Vector(float(self.gx.text() or 0), float(self.gy.text() or 0), float(self.gz.text() or 0))
            self.ground_generate_requested.emit(v)
        except ValueError:
            QMessageBox.warning(self, "Hata", "Lütfen koordinatları doğru girin.")
