#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path


def _find_latest_recording(out_dir: Path) -> Path | None:
    recordings = sorted(
        out_dir.glob("teleop-recording-*.py"),
        key=lambda p: p.stat().st_mtime,
    )
    return recordings[-1] if recordings else None


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    out_dir = repo_root / "out"
    latest = _find_latest_recording(out_dir)
    if latest is None:
        raise SystemExit(f"No teleop recordings found in {out_dir}")

    subprocess.run(
        [
            sys.executable,
            "-m",
            "pybricksdev",
            "run",
            "ble",
            "--name",
            "FatSean",
            str(latest),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
