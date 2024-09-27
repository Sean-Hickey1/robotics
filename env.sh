# skript to source a ros2 project
PROJECT_NAME=$(basename "$PWD")
if [ ! -f $PWD/.env.local ]; then
    echo "A 'env.local' will be created with the project name '$PROJECT_NAME'"
    echo "export ROS_DISTRO=jazzy                         # defines your ROS distro" > .env.local  
    echo "export PROJECT_NAME=$PROJECT_NAME               # defines your project name" >> .env.local
    echo 'export MR_DIR=${PROJECTS_DIR}/${PROJECT_NAME}   # defines your project root for MR_ROOT' >> .env.local
    echo 'export MR_WS=${MR_DIR}/ws02/src/mr              # defines your mr worksapce location' >> .env.local
    echo "export ROS_DOMAIN_ID=0                          # Can be used to avoid interference"  >> .env.local
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST  # nodes will only try to discover other nodes on the same machine "  >> .env.local 
fi
source $PWD/.env.local

#crate a persistent bash_history
[ ! -f $PWD/.devcontainer/bash_history ] && touch $PWD/.devcontainer/bash_history


export ROS_DISTRO=jazzy
export PROJECT_DIR=${MR_DIR}

if [ ! -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    echo "ROS_DISTRO '$ROS_DISTRO' not installed!"
    return 0  # Exit with a status code 0 (success)
fi

source_ws () {
    if [ -f "$1" ]; then
        source $1
        echo "sourced $1"
    else 
        echo "$1 does not exist."
    fi
}

echo "** ROS2 $ROS_DISTRO initialized with $RMW_IMPLEMENTATION **"
source /opt/ros/$ROS_DISTRO/setup.bash
source_ws ${PROJECT_DIR}/ws00/install/setup.bash
source_ws ${PROJECT_DIR}/ws01/install/setup.bash
source_ws ${PROJECT_DIR}/ws02/install/setup.bash
