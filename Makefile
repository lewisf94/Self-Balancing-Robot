# Convenience wrapper around colcon for the self-balancing robot workspace.
# Assumes ROS 2 is installed at /opt/ros/$(ROS_DISTRO). Override the distro with:
#   make build ROS_DISTRO=humble

ROS_DISTRO ?= jazzy
SHELL := /bin/bash
.ONESHELL:

.PHONY: help deps build test sim sim-rviz robot display clean

help:
	@echo "Self-Balancing Robot - make targets"
	@echo "  make deps      - install workspace dependencies with rosdep"
	@echo "  make build     - colcon build --symlink-install"
	@echo "  make test      - run the sbr_control unit tests"
	@echo "  make sim       - launch the Gazebo simulation"
	@echo "  make sim-rviz  - launch the simulation with RViz"
	@echo "  make robot     - bring up the real robot (Raspberry Pi path)"
	@echo "  make display   - view the URDF model in RViz"
	@echo "  make clean     - remove build/ install/ log/"

deps:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	rosdep install --from-paths src --ignore-src -r -y

build:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	colcon build --symlink-install

test:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	colcon test --packages-select sbr_control
	colcon test-result --verbose

sim:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	source install/setup.bash
	ros2 launch sbr_simulation simulation.launch.py

sim-rviz:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	source install/setup.bash
	ros2 launch sbr_simulation simulation.launch.py use_rviz:=true

robot:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	source install/setup.bash
	ros2 launch sbr_bringup robot.launch.py

display:
	source /opt/ros/$(ROS_DISTRO)/setup.bash
	source install/setup.bash
	ros2 launch sbr_description display.launch.py

clean:
	rm -rf build install log
