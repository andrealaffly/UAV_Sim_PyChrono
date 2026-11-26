"""Repo-root discovery shared by the SIL integration modules."""

import os
from pathlib import Path


def find_repo_root(start: Path) -> Path:
    """Repo root containing docker-compose.yml.

    Prefers the SIL_REPO_ROOT env var (set by the `ga_tuner` compose service to the
    host-absolute path the repo is mirrored at, for docker-outside-of-docker path
    resolution) since walking up from `start` doesn't work when this code runs inside
    a container whose /ros2_ws/src/simulator mount is a different subtree than
    wherever docker-compose.yml is mounted.
    """
    env_override = os.environ.get("SIL_REPO_ROOT")
    if env_override:
        repo_root = Path(env_override)
        if not (repo_root / "docker-compose.yml").is_file():
            raise FileNotFoundError(
                f"SIL_REPO_ROOT={env_override!r} does not contain docker-compose.yml"
            )
        return repo_root

    current = start.resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "docker-compose.yml").is_file():
            return candidate
    raise FileNotFoundError(
        f"Could not find docker-compose.yml above {start} — is the SIL integration "
        "code running inside the acsl_sil_ws repo checkout?"
    )
