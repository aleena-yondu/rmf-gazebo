

Fleet manager container (same `ROS_DOMAIN_ID`, host network, no test goal). The image build runs `colcon build` and does not start Gazebo; starting the container loads the world.

```
git clone git@github.com:aleena-yondu/rmf-gazebo.git
cd rmf-gazebo/src/r1_sim
docker/launch_docker.sh --build-image --gui --attach
```
