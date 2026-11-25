import hmac
from dataclasses import replace
from hashlib import sha256

import pytest

from app.comm.packet import DEV_KEY, FLAG_HEARTBEAT, Packet, decode, encode, load_swarm_key

KEY = bytes(range(32))
P = Packet(sender_id=3, seq=65535, t_ms=123456, pos=(12.3, -4.5, 50.0), vel=(1.25, 0.0, -0.5), state="Leader", flags=FLAG_HEARTBEAT)


def test_size():
    assert len(encode(P, KEY)) == 32


def test_roundtrip():
    q = decode(encode(P, KEY), KEY)
    assert q.sender_id == 3 and q.seq == 65535 and q.state == "Leader"
    assert q.pos == pytest.approx(P.pos, abs=0.05) and q.vel == pytest.approx(P.vel, abs=0.005)


def test_golden_body():
    assert encode(P, KEY)[:24].hex() == "0103ffff40e201007b00d3fff4017d000000ceff01010000"


def test_tamper_rejected():
    raw = bytearray(encode(P, KEY))
    raw[5] ^= 1
    assert decode(bytes(raw), KEY) is None


def test_wrong_key():
    assert decode(encode(P, KEY), b"x" * 32) is None


def test_bad_length():
    assert decode(b"\x00" * 31, KEY) is None


def test_bad_version():
    body = bytes([2]) + encode(P, KEY)[1:24]  # re-signed so only the version check can reject it
    assert decode(body + hmac.new(KEY, body, sha256).digest()[:8], KEY) is None


def test_clamp():
    assert decode(encode(replace(P, pos=(1e6, 0, 0)), KEY), KEY).pos[0] == pytest.approx(3276.7)


def test_key_env(monkeypatch):
    monkeypatch.setenv("SWARM_KEY", "00" * 32)
    assert load_swarm_key() == b"\x00" * 32
    monkeypatch.setenv("SWARM_KEY", "zz")
    pytest.raises(ValueError, load_swarm_key)


def test_short_key_rejected(monkeypatch):
    monkeypatch.setenv("SWARM_KEY", "00")
    with pytest.raises(ValueError, match="16 bytes"):
        load_swarm_key()
    monkeypatch.setenv("SWARM_KEY", "")
    assert load_swarm_key() == DEV_KEY
