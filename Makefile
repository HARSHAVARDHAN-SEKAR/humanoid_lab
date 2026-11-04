# Docker-first HUMANOID-LAB. All physics experiments have explicit scope.
IMAGE := humanoid-lab:humble
ARGS ?=
DOCKER_RUN := docker run --rm -it $(IMAGE)
DOCKER_RUN_GUI := docker run --rm -it -e DISPLAY=$(DISPLAY) -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw --network host $(IMAGE)

.PHONY: smoke help build test validate-supported shell stage1 stage2 stage3 stage4 stage5 stage1-gui stage2-gui stage3-gui tune

help:
	@echo 'make build | test | validate-supported | stage1..stage5 | stage3-gui | shell'
	@echo 'Default: fixed-torso, headless MuJoCo. Free-base: ARGS="model_file:=humanoid_biped.xml"'

build:
	docker build --build-arg ROS_DISTRO=humble -t $(IMAGE) .

test: build
	docker run --rm -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(IMAGE) python3 -m pytest -q -rs

validate-supported: build
	mkdir -p results/local
	docker run --rm -v "$(CURDIR)/results/local:/workspace/humanoid_lab_ws/results/local" $(IMAGE) \
		python3 -m humanoid_benchmarks.sim_validation --out results/local

tune: build
	docker run --rm $(IMAGE) python3 -m humanoid_benchmarks.tune_standing_gains --seconds 1.0

shell: build
	$(DOCKER_RUN) bash

stage1: build
	$(DOCKER_RUN) ros2 launch humanoid_bringup stage1_joint_control.launch.py use_rviz:=false use_joint_state_publisher_gui:=false $(ARGS)

stage2: build
	$(DOCKER_RUN) ros2 launch humanoid_bringup stage2_standing.launch.py use_rviz:=false use_joint_state_publisher_gui:=false $(ARGS)

stage3: build
	$(DOCKER_RUN) ros2 launch humanoid_bringup stage3_stepping.launch.py use_rviz:=false use_joint_state_publisher_gui:=false autostart_delay_s:=3.0 $(ARGS)

stage4: build
	$(DOCKER_RUN) ros2 launch humanoid_bringup stage4_walking.launch.py use_rviz:=false use_joint_state_publisher_gui:=false $(ARGS)

stage5: build
	$(DOCKER_RUN) ros2 launch humanoid_bringup stage5_disturbance_bench.launch.py use_rviz:=false use_joint_state_publisher_gui:=false $(ARGS)

stage1-gui: build
	$(DOCKER_RUN_GUI) ros2 launch humanoid_bringup stage1_joint_control.launch.py use_rviz:=true use_joint_state_publisher_gui:=false $(ARGS)

stage2-gui: build
	$(DOCKER_RUN_GUI) ros2 launch humanoid_bringup stage2_standing.launch.py use_rviz:=true use_joint_state_publisher_gui:=false $(ARGS)

stage3-gui: build
	$(DOCKER_RUN_GUI) ros2 launch humanoid_bringup stage3_stepping.launch.py use_rviz:=true use_joint_state_publisher_gui:=false autostart_delay_s:=3.0 $(ARGS)

smoke: build
	docker run --rm $(IMAGE) bash scripts/ros_smoke.sh
