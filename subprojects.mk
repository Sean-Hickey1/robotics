
demos-py:
		git clone git@git.auto.tuwien.ac.at:mr/demos-py.git $@

ws00/src/teleop_tools:
		git clone -b foxy-devel git@github.com:ros-teleop/teleop_tools.git $@
		touch $@/COLCON_IGNORE

ws00/src/slam_toolbox:
		git clone -b jazzy git@github.com:SteveMacenski/slam_toolbox.git $@
		touch $@/COLCON_IGNORE

ws00/src/navigation2:
		git clone -b jazzy git@github.com:ros-planning/navigation2.git $@
		touch $@/COLCON_IGNORE

ws01/src/Stage:
		git clone -b ros2 git@github.com:tuw-robotics/Stage.git $@

ws01/src/stage_ros2:
		git clone -b jazzy git@github.com:tuw-robotics/stage_ros2.git $@

ws01/src/tuw_geometry:
		git clone -b ros2 git@github.com:tuw-robotics/tuw_geometry.git $@

ws01/src/tuw_msgs:
		git clone -b ros2 git@github.com:tuw-robotics/tuw_msgs.git $@

ws01/src/tuw_laserscan_features:
		git clone -b ros2 git@github.com:tuw-robotics/tuw_laserscan_features.git $@

ws01/src/marker_msgs:
		git clone -b ros2 git@github.com:tuw-robotics/marker_msgs.git $@

ws02/src/mr:
		git clone -b main ssh://git@gitlab.tuwien.ac.at:822/lva-mr/2024/ws.git $@

clone-solutions:
		git clone -b main ssh://git@gitlab.tuwien.ac.at:822/lva-mr/2024/ws-solution.git ws02/src/mr
		

clone-ws00: \
	ws00/src/teleop_tools \
	ws00/src/slam_toolbox \
	ws00/src/navigation2

clone-ws01: \
	ws01/src/Stage \
	ws01/src/tuw_geometry  \
	ws01/src/stage_ros2  \
	ws01/src/marker_msgs \
	ws01/src/tuw_msgs \
	ws01/src/tuw_laserscan_features

clone-ws02: \
	ws02/src/mr

clone: clone-ws00 clone-ws01 clone-ws02 

pull:
	git pull origin
	find . -type d -name .git -exec echo {} \; -exec git --git-dir={} --work-tree=${PROJECT_DIR}/{}/.. pull origin \;