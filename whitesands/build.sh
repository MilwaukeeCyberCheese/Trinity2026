#!/bin/bash


mkdir builddir

uv export --format requirements-txt -o builddir/requirements.txt --no-hashes
cp setup.sh builddir
cp main.py builddir
pip download \
	--only-binary=:all: \
	--platform manylinux2_34_aarch64 \
	--python-version 3.13 \
	--implementation cp \
	--abi cp313 \
	-r builddir/requirements.txt \
	-d builddir/wheeldir
