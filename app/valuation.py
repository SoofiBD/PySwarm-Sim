"""
Target-revisit valuation, per Erkalkan et al. 2024 (Sensors 24(23):7859),
"Addressing the Return Visit Challenge in Autonomous FANETs".

Composite valuation U(u,g,t) = U1(t) * U2(u,g):
  U1(t): time-based urgency — linear growth, then quadratic past t2.
  U2(u,g): distance-based value — inverse distance (closer = higher).

Unlike the paper's revisit scenario, this project doesn't recycle visited
goals, so `age_ticks` measures "time waiting unassigned" rather than
"time since last visit" — same starvation problem, same fix. The paper's
t_threshold1 dead zone (U1=0 before t1) is dropped: with a single UAV/goal
pair it would stall assignment entirely, so U1 grows from t=0 instead.
"""


class TargetValuation:
    T_THRESHOLD_2 = 2.0   # seconds — urgency growth becomes quadratic past this
    EPSILON = 0.0001       # avoids divide-by-zero at distance 0

    @staticmethod
    def time_value(age_seconds: float) -> float:
        t2 = TargetValuation.T_THRESHOLD_2
        if age_seconds <= t2:
            return age_seconds
        return (age_seconds - t2) ** 2 + t2

    @staticmethod
    def distance_value(distance: float) -> float:
        return 1.0 / (distance + TargetValuation.EPSILON)

    @staticmethod
    def composite_value(age_seconds: float, distance: float) -> float:
        return TargetValuation.time_value(age_seconds) * TargetValuation.distance_value(distance)
