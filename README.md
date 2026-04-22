# Python Drone: FANET Test Environment

A modern, high-performance simulation environment for **Heterogeneous Flying Ad-hoc Networks (FANET)**. This project provides a robust platform for testing drone coordination, connectivity algorithms, and network routing in a 3D environment.

## 🚀 Key Features

- **3D Visualization**: Real-time rendering of UAVs and goals using Matplotlib's 3D backends.
- **Dynamic Simulation Engine**: Multi-threaded physics and logic engine for smooth simulation.
- **Modern UI**: Sleek, dark-themed interface built with **PySide6**.
- **Vectorized Computations**: High-performance mathematical operations leveraging **NumPy**.
- **State Management**: Robust data modeling using **Pydantic**.
- **Connectivity Analysis**: Real-time adjacency matrix and connected components tracking.
- **Collision Avoidance**: Drones detect and avoid collisions with nearby UAVs.
- **Drone-to-Drone Communication**: UAVs share target info with neighbors within communication range.
- **Relay Assignment**: Automatic relay drone assignment when network connectivity is threatened.
- **Structured Logging**: Event-based logging for simulation debugging.

## 🛠 Technology Stack

- **GUI**: PySide6 (Qt for Python)
- **Computation**: NumPy
- **Visualization**: Matplotlib (Qt6/Agg backend)
- **Data Validation**: Pydantic v2
- **Testing**: pytest
- **Language**: Python 3.9+

## 📥 Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/SoofiBD/PythonDrone-master.git
   cd PythonDrone-master
   ```

2. **Set up a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # Mac/Linux
   # or
   venv\Scripts\activate     # Windows
   ```

3. **Install dependencies**:
   ```bash
   pip install .
   ```

4. **Install dev dependencies** (optional, for testing):
   ```bash
   pip install pytest
   ```

## 🚦 How to Run

After setting up the environment, simply run:

```bash
python run.py
```

### Controls:
- **Gen UAVs**: Generate a specified number of drones.
- **Gen Goals**: Generate target points for drones to visit.
- **Set Ground**: Set the base station (Ground) coordinates.
- **Start/Stop**: Control the simulation state.
- **Reset**: Clear the environment and reset drone states.

## 🧪 Running Tests

```bash
pytest tests/ -v
```

## 📂 Project Structure

- `run.py`: Entry point of the application.
- `View.py`: GUI definition and visualization logic.
- `app/`: Core logic and data models.
  - `engine.py`: Simulation thread and step logic.
  - `uav.py`, `goal.py`, `ground.py`: Entity models.
  - `manage_drones.py`: High-level coordination algorithms.
  - `logger.py`: Structured logging.
  - `myMath/`: Optimized mathematical libraries.
- `tests/`: Unit tests.

## 🔧 Configuration

Key parameters in `app/manage_drones.py`:
- `CollisionDetector.MIN_SAFE_DISTANCE = 5.0`: Minimum distance to avoid collisions
- `DroneNetwork.COMM_RANGE = 150.0`: Drone-to-drone communication range

Key parameters in `app/engine.py`:
- `self.cthr = 200.0`: Connectivity threshold for network components
- `self.interval = 0.05`: Simulation step interval (50ms)

## 📝 Logging

Simulation events are logged to:
- Console (stdout)
- `logs/simulation.log`

Log event types:
- `[CONNECTIVITY]`: Network component detection
- `[RELAY]`: Relay assignment
- `[NETWORK]`: Drone-to-drone links
- `[COMM]`: Target sharing

---
*Developed as part of modernizing FANET research tools.*