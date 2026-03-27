#!/bin/bash


python3 -m venv .venv
source .venv/bin/activate

pip install --no-index --find-links=./wheeldir -r requirements.txt
