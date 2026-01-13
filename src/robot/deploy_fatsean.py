#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    main_file = repo_root / "src" / "robot" / "main.py"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pybricksdev",
            "run",
            "ble",
            "--name",
            "FatSean",
            str(main_file),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
