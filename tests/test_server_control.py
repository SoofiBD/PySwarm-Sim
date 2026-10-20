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
