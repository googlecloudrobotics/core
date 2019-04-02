FROM osrf/ros:lunar-desktop-full-xenial

RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    ros-lunar-map-server \
    ros-lunar-move-base \
    ros-lunar-amcl \
    ros-lunar-dwa-local-planner \
       && apt-get clean \
       && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src/ && cd $CATKIN_WS/src/
WORKDIR $CATKIN_WS/src
ADD ros $CATKIN_WS/src/
RUN git clone https://www.github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone https://www.github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone https://www.github.com/ROBOTIS-GIT/turtlebot3_simulations.git

ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
WORKDIR $CATKIN_WS
RUN /bin/bash -c ". /opt/ros/lunar/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release"

ADD ros/data/map.pgm /root/map.pgm
ADD ros/data/map.yaml /root/map.yaml

RUN hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models

ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]

