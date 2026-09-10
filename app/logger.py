import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path

LOG_DIR = Path(__file__).parent.parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

_handler_file = RotatingFileHandler(
    LOG_DIR / "simulation.log",
    maxBytes=10 * 1024 * 1024,
    backupCount=5,
    encoding="utf-8",
)
_handler_stdout = logging.StreamHandler(sys.stdout)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
    handlers=[_handler_stdout, _handler_file],
)

logger = logging.getLogger("DroneSimulation")


def log_simulation_event(event_type: str, message: str) -> None:
    logger.info(f"[{event_type}] {message}")


def log_error(component: str, message: str) -> None:
    logger.error(f"[{component}] {message}")


def log_warning(component: str, message: str) -> None:
    logger.warning(f"[{component}] {message}")
