import hmac
import os
import struct
from dataclasses import dataclass
from hashlib import sha256

from app.logger import log_warning

PACKET_SIZE = 32
BODY_SIZE = 24
PROTOCOL_VERSION = 1
STATE_CODES = {"Free": 0, "Leader": 1, "Slave": 2}
FLAG_HEARTBEAT = 0x01
DEV_KEY = b"dev-swarm-key-not-for-flight!!!!"
MIN_KEY_BYTES = 16

_BODY = struct.Struct("<BBHI3h3hBBH")  # ver, sender, seq, t_ms, pos dm, vel cm/s, state, flags, reserved
_STATE_NAMES = {v: k for k, v in STATE_CODES.items()}
_warned = False


@dataclass(frozen=True)
class Packet:
    sender_id: int
    seq: int
    t_ms: int
    pos: tuple[float, float, float]
    vel: tuple[float, float, float]
    state: str
    flags: int = 0


def _i16(value: float) -> int:
    return max(-32768, min(32767, round(value)))


def _mac(body: bytes, key: bytes) -> bytes:
    return hmac.new(key, body, sha256).digest()[:8]


def encode(p: Packet, key: bytes) -> bytes:
    body = _BODY.pack(
        PROTOCOL_VERSION,
        p.sender_id,
        p.seq & 0xFFFF,
        p.t_ms & 0xFFFFFFFF,
        *(_i16(c * 10) for c in p.pos),
        *(_i16(c * 100) for c in p.vel),
        STATE_CODES[p.state],
        p.flags,
        0,
    )
    return body + _mac(body, key)


def decode(raw: bytes, key: bytes) -> Packet | None:
    if len(raw) != PACKET_SIZE:
        return None
    body, tag = raw[:BODY_SIZE], raw[BODY_SIZE:]
    if not hmac.compare_digest(tag, _mac(body, key)):
        return None
    ver, sender, seq, t_ms, px, py, pz, vx, vy, vz, state, flags, _ = _BODY.unpack(body)
    if ver != PROTOCOL_VERSION or state not in _STATE_NAMES:
        return None
    return Packet(
        sender, seq, t_ms,
        (px / 10, py / 10, pz / 10),
        (vx / 100, vy / 100, vz / 100),
        _STATE_NAMES[state], flags,
    )


def load_swarm_key() -> bytes:
    global _warned
    hex_key = os.environ.get("SWARM_KEY")
    if not hex_key:  # unset or empty (compose passes an empty string)
        if not _warned:
            log_warning("comm", "SWARM_KEY not set; using insecure development key")
            _warned = True
        return DEV_KEY
    key = bytes.fromhex(hex_key)
    if len(key) < MIN_KEY_BYTES:
        raise ValueError(f"SWARM_KEY must be at least {MIN_KEY_BYTES} bytes ({2 * MIN_KEY_BYTES} hex chars), got {len(key)}")
    return key
