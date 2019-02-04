# Docker script to run a headless turtlebot3 gazebo simulation

The container executes a roslaunch file that brings up
- turtlebot3_bringup
- amcl navigation with move_base
- map server
- headless gazebo with the turtlebot3 world

TODO(jkammerl) The map in "ros" folder is a placeholder and should be rebuilt.

## Build
docker build -t turtlebot3_gazebo_headless .

## Run (as daemon)
docker run -d --network host  turtlebot3_gazebo_headless:latest


