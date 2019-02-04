<!--
  A simple roslaunch file that just starts a single test node. When rostest
  launches this, it will start a roscore and then the test node.
-->

<launch>
  <test pkg="${ROSPKG}" test-name="${NAME}" type="${NAME}_node" />
</launch>
