import logging
import random

from app.ground import Ground
from app.myMath.matrixOperation import MatrixOperation
from app.myMath.vector import Vector
from app.uav import UAV

logger = logging.getLogger("DroneSimulation")

_MAX_ATTEMPTS = 500


def _try_place_near(ground: Ground, cthr: float) -> Vector | None:
    for _ in range(_MAX_ATTEMPTS):
        x = random.randrange(0, 800)
        y = random.randrange(0, 800)
        if MatrixOperation.distance_XYZ(x, y, 0, ground.getPos().getX(), ground.getPos().getY(), ground.getPos().getZ()) < (cthr * 0.9):
            return Vector(x, y, 0)
    return None


def _try_place_near_uav(anchor: Vector, cthr: float) -> Vector | None:
    for _ in range(_MAX_ATTEMPTS):
        x = random.randrange(0, 800)
        y = random.randrange(0, 800)
        if MatrixOperation.distance_XYZ(x, y, 0, anchor.getX(), anchor.getY(), anchor.getZ()) < (cthr * 0.9):
            return Vector(x, y, 0)
    return None


class GenerateUAV:

    @staticmethod
    def Run(count: int = None, loctype: int = None, cthr: float = None, UAVs: list[UAV] | None = None, ground: Ground = None):
        if UAVs is None:
            UAVs = []
        if count is None:
            count = 0
        if cthr is None:
            cthr = 200.0
        target_len = len(UAVs) + count

        # If ground exists, ensure at least one UAV near ground
        if ground is not None and not any(
            MatrixOperation.distance(u.pos, ground.getPos()) < cthr * 0.9 for u in UAVs
        ):
            pos = _try_place_near(ground, cthr)
            if pos is None:
                logger.warning("GenerateUAV: could not place UAV near ground after %d attempts, using random", _MAX_ATTEMPTS)
                pos = Vector(random.randrange(0, 800), random.randrange(0, 800), 0)
            uav = UAV(pos=pos, cthr=cthr, ground=ground)
            uav.setUAVNo(len(UAVs) + 1)
            uav.setIPAddress(f"10.0.0.{len(UAVs) + 2}")
            UAVs.append(uav)

        for i in range(len(UAVs), target_len):
            pos: Vector | None = None
            if loctype == 1 and i > 0:
                anchor_idx = 0 if i == 1 else random.randrange(0, i - 1)
                # anchors are within UAVs list (may be empty for first iter already handled)
                anchor = UAVs[anchor_idx].getPos() if anchor_idx < len(UAVs) else None
                if anchor is not None:
                    pos = _try_place_near_uav(anchor, cthr)
            elif loctype != 1 and UAVs:
                pos = _try_place_near_uav(UAVs[0].getPos(), cthr)

            if pos is None:
                if i > 0 and pos is None and loctype == 1:
                    logger.warning("GenerateUAV: fallback random placement for index %d", i)
                pos = Vector(random.randrange(0, 800), random.randrange(0, 800), 0)

            uav = UAV(pos=pos, cthr=cthr, ground=ground)
            uav.setUAVNo(len(UAVs) + 1)
            uav.setIPAddress(f"10.0.0.{len(UAVs) + 2}")
            UAVs.append(uav)

        return UAVs
