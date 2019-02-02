#!/bin/bash

while (! docker stats --no-stream); do
  echo "Waiting for Docker to launch..."
  sleep 1
done

docker run \
    --rm \
    --privileged \
    --net foo \
    --name master \
    -v /home/pi/.ros/:/root/.ros/ \
    -v /home/pi/catkin_ws/:/root/catkin_ws/ \
    -p 9090:9090 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    test-ros
