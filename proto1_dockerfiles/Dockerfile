#FROM osrf/ros:kinetic-desktop-full
FROM ubuntu:16.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get -y install \
    lsb-release

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update && apt-get -y install \
    ros-kinetic-desktop \
    ros-kinetic-cv-camera \
    ros-kinetic-compressed-image-transport \
    ros-kinetic-rosbridge-server \
    ros-kinetic-tf

WORKDIR /root/

RUN apt-get install -y \
    python-pip python-dev \
    git-all

RUN pip install \
    smbus2

RUN git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git && \
    cd Adafruit_Python_PCA9685 && \
    python setup.py install

WORKDIR /root/catkin_ws/

CMD source /opt/ros/kinetic/setup.bash && \
    catkin_make && \
    source /root/catkin_ws/devel/setup.bash && \
    roslaunch proto1_ros bringup.launch
#    roslaunch rosbridge_server rosbridge_websocket.launch

