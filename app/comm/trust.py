class TrustTable:
    """Per-receiver EMA trust score for each peer, with quorum quarantine."""

    def __init__(
        self,
        alpha: float = 0.05,
        init: float = 0.5,
        trust_min: float = 0.7,
        pred_err_ref: float = 5.0,
        rssi_z_max: float = 2.0,
    ) -> None:
        self.alpha = alpha
        self.init = init
        self.trust_min = trust_min
        self.pred_err_ref = pred_err_ref
        self.rssi_z_max = rssi_z_max
        self._score: dict[tuple[int, int], float] = {}

    def get(self, receiver: int, peer: int) -> float:
        return self._score.get((receiver, peer), self.init)

    def observe(self, receiver: int, peer: int, pred_residual_m: float, rssi_z: float) -> None:
        good = pred_residual_m <= 3 * self.pred_err_ref and abs(rssi_z) <= self.rssi_z_max
        s = self.get(receiver, peer)
        self._score[(receiver, peer)] = s + self.alpha * ((1.0 if good else 0.0) - s)

    def penalize(self, receiver: int, peer: int, amount: float = 0.2) -> None:
        self._score[(receiver, peer)] = max(0.0, self.get(receiver, peer) - amount)

    def _low(self, peer: int, receivers: set[int]) -> int:
        return sum(self.get(r, peer) < self.trust_min for r in receivers)

    def quarantined(self, hearers: dict[int, set[int]]) -> set[int]:
        return {
            p for p, rs in hearers.items()
            if len(rs) >= 2 and 2 * self._low(p, rs) > len(rs)
        }

    def suspect(self, hearers: dict[int, set[int]]) -> set[int]:
        q = self.quarantined(hearers)
        return {p for p, rs in hearers.items() if p not in q and self._low(p, rs) > 0}

    def mean_trust(self, peer: int) -> float:
        v = [s for (_, p), s in self._score.items() if p == peer]
        return sum(v) / len(v) if v else 1.0
