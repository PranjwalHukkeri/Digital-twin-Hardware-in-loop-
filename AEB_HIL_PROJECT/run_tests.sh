#!/usr/bin/env bash
# Helper script to run unit tests without ROS plugin interference
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH=.
python3 -m pytest tests/ -v "$@"
