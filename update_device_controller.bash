#!/bin/bash
git pull origin main
colcon build --packages-select device_controller && source install/local_setup.bash
