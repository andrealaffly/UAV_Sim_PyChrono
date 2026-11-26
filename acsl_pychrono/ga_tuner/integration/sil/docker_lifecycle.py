"""Restart the flightstack container with freshly-written gains, per GA candidate.

A genuine process restart (not just a gains re-read) is required for every
candidate: is_offset_odometry_msg_ (vehicle_state.cpp, default True) and
timestamp_initial_ (multi_threaded_node.cpp) are one-shot C++ node state that
only reset to fresh defaults when the process itself restarts. We verify this
actually happened via the container's StartedAt timestamp rather than trusting
`docker compose restart` blindly.
"""

import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from . import gain_serializer

_HEALTH_POLL_INTERVAL_S = 2.0


class FlightstackRestartError(RuntimeError):
    """Raised when the flightstack container fails to restart, or is misconfigured."""


def _run(args, repo_root: Path) -> str:
    result = subprocess.run(
        args, cwd=repo_root, check=True, capture_output=True, text=True,
    )
    return result.stdout.strip()


def _container_id(repo_root: Path) -> str:
    container_id = _run(["docker", "compose", "ps", "-q", "flightstack"], repo_root)
    if not container_id:
        raise FlightstackRestartError(
            "docker compose ps -q flightstack returned no container id — "
            "was `docker compose up -d flightstack` run first?"
        )
    return container_id


def _inspect(container_id: str, fmt: str, repo_root: Path) -> str:
    return _run(["docker", "inspect", "--format", fmt, container_id], repo_root)


def _assert_controller_type(container_id: str, expected: str, repo_root: Path) -> None:
    actual = _run(
        ["docker", "compose", "exec", "-T", "flightstack", "printenv", "CONTROLLER_TYPE"],
        repo_root,
    )
    if actual != expected:
        raise FlightstackRestartError(
            f"flightstack container is running CONTROLLER_TYPE={actual!r}, expected "
            f"{expected!r}. Set docker-compose.yml's x-controller-type anchor to "
            f"{expected!r} before running SIL-GA tuning."
        )


def _wait_healthy(container_id: str, repo_root: Path, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_status = None
    while time.monotonic() < deadline:
        last_status = _inspect(container_id, "{{.State.Health.Status}}", repo_root)
        if last_status == "healthy":
            return
        time.sleep(_HEALTH_POLL_INTERVAL_S)
    raise FlightstackRestartError(
        f"flightstack container did not become healthy within {timeout_s}s "
        f"(last status: {last_status!r})"
    )


def restart_flightstack_with_gains(
    gain_dict: Dict[str, Any],
    repo_root: Path,
    controller_type: str,
    tuned_parameter_names: Optional[Sequence[str]] = None,
    timeout_s: float = 180.0,
) -> None:
    """Write gain_dict into gains_<controller_type>.json and restart flightstack with it.

    tuned_parameter_names restricts the write to only the gain groups actually being
    tuned (see gain_serializer.write_pid_gains_json) so untouched groups (e.g. rotational,
    when only translational is being tuned) aren't silently overwritten by the GA's
    Python-side template defaults.
    """
    if controller_type != "PID":
        raise NotImplementedError(
            f"SIL-GA restart-per-candidate is only wired up for PID so far, got "
            f"{controller_type!r}."
        )

    gains_path = repo_root / "src" / "flightstack" / "params" / "control" / "pid" / "gains_pid.json"
    gain_serializer.write_pid_gains_json(gain_dict, gains_path, tuned_parameter_names)

    # Idempotent: creates+starts the container if it doesn't exist yet, otherwise a no-op
    # (compose sees no config change from a bind-mounted file edit). The `restart` call
    # below is what actually guarantees a fresh process on every single invocation.
    _run(["docker", "compose", "up", "-d", "flightstack"], repo_root)

    container_id = _container_id(repo_root)
    started_before = _inspect(container_id, "{{.State.StartedAt}}", repo_root)

    _run(["docker", "compose", "restart", "flightstack"], repo_root)

    # `restart` recreates the process but keeps the same container id, so re-query by id.
    started_after = _inspect(container_id, "{{.State.StartedAt}}", repo_root)
    if started_after == started_before:
        raise FlightstackRestartError(
            "flightstack container's StartedAt timestamp did not change after "
            "`docker compose restart` — the process did not actually restart, so "
            "one-shot state (odometry zero-offset, initial timestamp) from the "
            "previous candidate would still be in effect."
        )

    _wait_healthy(container_id, repo_root, timeout_s)
    _assert_controller_type(container_id, controller_type, repo_root)
