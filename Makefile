# Check if the environment variable MY_VAR is set
ifeq ($(PROJECT_NAME),)
$(error Project not sourced! run 'source env.sh' in your projcet folder.)
endif

PROJECT_DIR = $(shell pwd)
SHELL = /usr/bin/env bash
DOCKER_OWNER = tuwrobotics
DOCKER_IMAGE_BASE=mr2024-base
BUILD_TYPE = Debug
include *.mk

all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   make clean-all         - removes from all ws install, build, log"
	@echo "   make clean-ws00        - removes from ws00 install, build, log"
	@echo "   make clean-ws01        - removes from ws01 install, build, log"
	@echo "   make clean-ws02        - removes from ws02 install, build, log"
	@echo "   make clone-ws00        - clones the subrepostories needed in ws00"
	@echo "   make clone-ws01        - clones the subrepostories needed in ws01"
	@echo "   make clone-ws02        - clones the subrepostories needed in ws02"
	@echo "   make build             - builds workspace ws02"
	@echo "   make build-all         - builds workspace ws00, ws01, ws02"
	@echo "   make build-ws00        - builds workspace ws00"
	@echo "   make build-ws01        - builds workspace ws01"
	@echo "   make build-ws02        - builds workspace ws02"
	@echo "   make docker-attach     - attaches the bash to the devcontainer"
	@echo ""
	@echo "   For internal use:"
	@echo ""
	@echo "   make docker-build-base     - builds the base container used by the devcontainer"
	@echo "   make docker-login USER=xy  - authenticate container Registry"
	@echo "   make docker-push-base      - pushes the base container to dockerhub"
	@echo ""


docker-attach:
	@docker exec -it -e TERM=xterm-256color -w ${PROJECT_DIR} ${PROJECT_NAME} bash

docker-build-base:
	@docker build --rm -t ${DOCKER_OWNER}/${DOCKER_IMAGE_BASE}  -f .devcontainer/Dockerfile-base .devcontainer/.

docker-push-base:
	@docker push ${DOCKER_OWNER}/${DOCKER_IMAGE_BASE}

docker-list-containers:
	@docker ps -a  --format "table {{.Names}}\t{{.ID}}\t{{.Status}}"
	
docker-remove-container:
	@docker stop ${PROJECT_NAME}; docker rm ${PROJECT_NAME}

docker-remove-images:
	@docker image rm -f $$(docker images -a -q --filter "reference=*${PROJECT_NAME}*")
	
docker-remove-base:
	@docker image rm -f $$(docker images -a -q --filter "reference=*${DOCKER_IMAGE_BASE}*")
	
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

