#!/bin/bash

# Allow local GUI applications
xhost +

# Run the container with GUI and volume support
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/ros2_ws" \
    calib_container

