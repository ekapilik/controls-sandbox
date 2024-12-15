#!/usr/bin/bash
python3 -m venv sandbox
source ./sandbox/bin/activate
pip install -r ./requirements.txt

pybind11-stubgen mujoco -o ./.stubs
pybind11-stubgen cv2 -o ./.stubs
