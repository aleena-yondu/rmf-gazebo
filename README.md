

Fleet manager container (same `ROS_DOMAIN_ID`, host network, no test goal). The image build runs `colcon build` and does not start Gazebo; starting the container loads the world.

```
git clone git@github.com:aleena-yondu/rmf-gazebo.git
cd rmf-gazebo/src/r1_sim
docker/launch_docker.sh --build-image --gui --attach
```

Send a Nav2 goal from the terminal once the log shows `Managed nodes are active`:

```
docker exec gazebo_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: -4.0}, orientation: {w: 1.0}}}}' --feedback"
```

The `map` frame is the Gazebo world frame, so a Gazebo position is also a valid goal. Valid x is -9.63 to 10.21 and y is -9.73 to 9.81; anything outside fails with `Goal Coordinates ... was outside bounds`. Keep goals about 0.6 m clear of obstacles. The conveyor at y = -1.7, x = -5 to 5 has a bar above the lidar, so it is marked occupied on the map. To read a valid pose, click Goal Pose in RViz while running:

```
docker exec gazebo_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /goal_pose --once"
```
