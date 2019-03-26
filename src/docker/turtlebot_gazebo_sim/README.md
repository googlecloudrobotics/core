# Docker script to run a headless turtlebot3 gazebo simulation

The container executes a roslaunch file that brings up
- turtlebot3_bringup
- amcl navigation with move_base
- map server
- headless gazebo with the turtlebot3 world

TODO(jkammerl) The map in "ros" folder is a placeholder and should be rebuilt.

## Build
./deploy.sh <container-registry>/<project-id>

## Run (as daemon)
For the <git-sha> see the output at the end of the previous build step.
docker run -d --network host turtlebot3-gazebo-headless/<git-sha>
