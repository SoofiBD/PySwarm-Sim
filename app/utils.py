import json
import logging
from pathlib import Path

from app.logger import log_error

logger = logging.getLogger("DroneSimulation")


class Utils:

    @staticmethod
    def writeToFile(content: str, filePath: str) -> None:
        try:
            Path(filePath).parent.mkdir(parents=True, exist_ok=True)
            with open(filePath, "a", encoding="utf-8") as f:
                f.write(content)
        except Exception as e:
            log_error("Utils.writeToFile", f"{filePath}: {e}")
            logger.exception("writeToFile failed")

    @staticmethod
    def readJSON(filePath: str) -> dict | list | None:
        try:
            with open(filePath, encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            log_error("Utils.readJSON", f"not found: {filePath}")
            return None
        except json.JSONDecodeError as e:
            log_error("Utils.readJSON", f"JSON error {filePath}: {e}")
            return None
        except Exception as e:
            log_error("Utils.readJSON", f"{filePath}: {e}")
            logger.exception("readJSON failed")
            return None
