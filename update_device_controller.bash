#!/bin/bash
colcon build --packages-select device_controller && source install/local_setup.bash
