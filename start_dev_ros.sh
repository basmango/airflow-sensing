
# Get the current directory
CURRENT_DIR=$(pwd)

# Start the ROS Noetic Desktop Full container with the current directory mounted
docker run -it --rm \
    --net=host \
    --privileged \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$CURRENT_DIR:$CURRENT_DIR" \
    --workdir="$CURRENT_DIR" \
    osrf/ros:noetic-desktop-full \
    bash 

