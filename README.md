```
ros2 launch r1_sim goal_sim_ground_truth.launch.py
```

Fleet manager container (same `ROS_DOMAIN_ID`, host network, no test goal):

```
cd src/r1_sim
docker/launch_docker.sh --build-image
```
