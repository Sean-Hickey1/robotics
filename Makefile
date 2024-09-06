# Check if the environment variable MY_VAR is set
ifeq ($(PROJECT_NAME),)
$(error Project not sourced! run 'source env.sh' in your projcet folder.)
endif

PROJECT_DIR = $(shell pwd)
SHELL = /usr/bin/env bash
BUILD_TYPE = Debug
include *.mk

all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   make release           - builds all workspaces in release"
	@echo "   make clean             - removes from workspaces install, build, log"
	@echo "   make docker-build-base - builds a container usid in .devcontainer"
	@echo "   make clone             - clones the subrepostories needed"
	@echo ""


docker-attach:
	@docker exec -it -e TERM=xterm-256color -w ${PROJECT_DIR} ${PROJECT_NAME} bash

docker-build-base:
	@docker build --rm -t jazzy-base-dev  -f .devcontainer/Dockerfile-base .devcontainer/.

docker-list-containers:
	@docker ps -a  --format "table {{.Names}}\t{{.ID}}\t{{.Status}}"
	
docker-remove-container:
	@docker stop ${PROJECT_NAME}; docker rm ${PROJECT_NAME}

docker-remove-images:
	@docker image rm -f $$(docker images -a -q --filter "reference=*${PROJECT_NAME}*")
	
docker-remove-base:
	@docker image rm -f $$(docker images -a -q --filter "reference=*jazzy-base-dev*")
	
docker-prune-none:
	docker image prune
	
clean-ws00:
	rm -rf ./ws00/install ./ws00/build  ./ws00/log

clean-ws01:
	rm -rf ./ws01/install ./ws01/build  ./ws01/log

clean-ws02:
	rm -rf ./ws02/install ./ws02/build  ./ws02/log

clean-all: clean-ws00 clean-ws01 clean-ws02
	rm -rf ./install ./build  ./log

delete-packages:
	rm -rf ./ws00/src/*
	rm -rf ./ws01/src/*
	rm -rf ./ws02/src/*

build:  \
	build-ws00 \
	build-ws01 \
	build-ws02 \

build-ws00: 
	cd ${PROJECT_DIR}/ws00; \
	source /opt/ros/${ROS_DISTRO}/setup.bash; \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}; \

build-ws01: 
	cd ${PROJECT_DIR}/ws01; \
	source /opt/ros/${ROS_DISTRO}/setup.bash; \
	source ${PROJECT_DIR}/ws00/install/setup.bash; \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} --packages-select Stage --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY; \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

build-ws02: 
	source /opt/ros/${ROS_DISTRO}/setup.bash; \
	source ${PROJECT_DIR}/ws00/install/setup.bash; \
	source ${PROJECT_DIR}/ws01/install/setup.bash; \
	cd ${PROJECT_DIR}/ws02; \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

