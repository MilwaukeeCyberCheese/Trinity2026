#!/bin/bash

set -euo pipefail

TARGET_PLATFORM="${TARGET_PLATFORM:-manylinux2014_aarch64}"
TARGET_PYTHON_VERSION="${TARGET_PYTHON_VERSION:-3.13}"
TARGET_PYTHON_NODOT="${TARGET_PYTHON_VERSION//./}"

rm -rf builddir
mkdir -p builddir/wheeldir

cp setup.sh builddir
cp main.py builddir
printf '%s\n' \
	'setuptools>=75.0.0' \
	'wheel>=0.45.0' \
	'adafruit-circuitpython-neopixel>=6.4.0' \
	'pynetworktables>=2021.0.0' \
	> builddir/requirements.txt

printf '%s\n' \
	'sysv_ipc>=1.1.0' \
	'rpi-ws281x>=5.0.0' \
	> builddir/source-requirements.txt

pip download \
	--only-binary=:all: \
	--platform "$TARGET_PLATFORM" \
	--python-version "$TARGET_PYTHON_VERSION" \
	--implementation cp \
	--abi "cp${TARGET_PYTHON_NODOT}" \
	-r builddir/requirements.txt \
	-d builddir/wheeldir

pip download \
	--no-binary=:all: \
	--no-deps \
	--dest builddir/wheeldir \
	-r builddir/source-requirements.txt
