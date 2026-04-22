import logging
import sys
from pathlib import Path

LOG_DIR = Path(__file__).parent.parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(LOG_DIR / "simulation.log"),
    ],
)

logger = logging.getLogger("DroneSimulation")

def log_simulation_event(event_type: str, message: str) -> None:
    logger.info(f"[{event_type}] {message}")

def log_error(component: str, message: str) -> None:
    logger.error(f"[{component}] {message}")

def log_warning(component: str, message: str) -> None:
    logger.warning(f"[{component}] {message}")