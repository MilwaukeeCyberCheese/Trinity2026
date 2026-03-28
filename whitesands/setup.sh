#!/bin/bash

set -euo pipefail

python3 -m venv --system-site-packages .venv
source .venv/bin/activate

pip install --no-index --find-links=./wheeldir setuptools wheel
pip install --no-index --no-build-isolation --find-links=./wheeldir -r requirements.txt
if [[ -f source-requirements.txt ]]; then
	pip install --no-index --no-build-isolation --find-links=./wheeldir -r source-requirements.txt
fi

python3 - <<'PY'
import importlib
import sys

required = [
    ("RPi.GPIO", "python3-rpi.gpio"),
    ("_rpi_ws281x", "the bundled rpi-ws281x package or a system package providing it"),
]

missing = []
for module_name, package_name in required:
    try:
        importlib.import_module(module_name)
    except ModuleNotFoundError:
        missing.append((module_name, package_name))

if missing:
    for module_name, package_name in missing:
        print(
            f"Missing system module {module_name}. Install Raspberry Pi OS package: {package_name}",
            file=sys.stderr,
        )
    sys.exit(1)
PY
