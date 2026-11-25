from fastapi.testclient import TestClient

import server.main as main


def test_stop_cancels_loop_so_start_runs_only_one():
    with TestClient(main.app) as client:
        first = main._runner_task
        client.post("/api/control", json={"action": "stop"})
        assert first.done()
        client.post("/api/control", json={"action": "start"})
        assert main._runner_task is not None and main._runner_task is not first


def test_admin_token_required_when_set(monkeypatch):
    monkeypatch.setenv("ADMIN_TOKEN", "s3cret")
    with TestClient(main.app) as client:
        assert client.post("/api/control", json={"action": "stop"}).status_code == 401
        ok = client.post("/api/control", json={"action": "stop"},
                         headers={"X-Admin-Token": "s3cret"})
        assert ok.status_code == 200


def _set_mode(client, mode="predictive"):
    return client.post("/api/config", json={"n_drones": 6, "n_targets": 8, "comm_mode": mode})


def test_config_comm_mode_shows_in_metrics():
    with TestClient(main.app) as client:
        assert _set_mode(client).status_code == 200
        assert client.get("/api/metrics").json()["comm"]["comm_mode"] == "predictive"


def test_config_rejects_invalid_comm_mode():
    with TestClient(main.app) as client:
        assert _set_mode(client, "bogus").status_code == 422


def test_reset_keeps_comm_mode():
    with TestClient(main.app) as client:
        _set_mode(client)
        client.post("/api/control", json={"action": "reset"})
        assert client.get("/api/metrics").json()["comm"]["comm_mode"] == "predictive"


def test_env_comm_mode_invalid_falls_back_to_ideal(monkeypatch):
    monkeypatch.setenv("COMM_MODE", "nope")
    with TestClient(main.app) as client:
        assert "comm" not in client.get("/api/metrics").json()


def test_fault_requires_token_when_set(monkeypatch):
    monkeypatch.setenv("ADMIN_TOKEN", "s3cret")
    with TestClient(main.app) as client:
        r = client.post("/api/fault", json={"kind": "gps_spoof", "drone_id": 1})
        assert r.status_code == 401


def test_fault_409_in_ideal_mode():
    with TestClient(main.app) as client:
        r = client.post("/api/fault", json={"kind": "gps_spoof", "drone_id": 1})
        assert r.status_code == 409


def test_fault_unknown_drone_404():
    with TestClient(main.app) as client:
        _set_mode(client)
        r = client.post("/api/fault", json={"kind": "gps_spoof", "drone_id": 999})
        assert r.status_code == 404


def test_fault_spoof_and_replay_inject():
    with TestClient(main.app) as client:
        _set_mode(client)
        assert client.post("/api/fault", json={"kind": "gps_spoof", "drone_id": 1}).json() == {"status": "injected"}
        assert 1 in main.runner.comm.spoofs
        assert client.post("/api/fault", json={"kind": "replay", "drone_id": None}).status_code == 200
        assert main.runner.comm.replay_pos is not None
