

Fleet manager container (same `ROS_DOMAIN_ID`, host network, no test goal). The image build runs `colcon build` and does not start Gazebo; starting the container loads the world.

```
git clone git@github.com:aleena-yondu/rmf-gazebo.git
cd rmf-gazebo/src/r1_sim
docker/launch_docker.sh --build-image --gui --attach
```

Send a Nav2 goal from the terminal once the log shows `Managed nodes are active`:

```
docker exec gazebo_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: -8.5, y: 26.5}, orientation: {w: 1.0}}}}' --feedback"
```

The goal is in the `map` frame, not Gazebo world coordinates. Valid map x is -21.43 to -2.53 and y is 16.51 to 36.46; anything outside fails with `Goal Coordinates ... was outside bounds`. To read a valid pose, click Goal Pose in RViz while running:

```
docker exec gazebo_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /goal_pose --once"
```
