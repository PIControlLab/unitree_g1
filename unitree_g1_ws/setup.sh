#!/bin/bash

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
