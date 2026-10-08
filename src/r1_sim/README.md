# r1_sim

Ground-truth Nav2 simulation of the warehouse robot for the fleet manager. The image is ROS 2 Jazzy with Gazebo Sim 8.

The fleet manager and this container share the host network and must use the same `ROS_DOMAIN_ID`. Headless mode does not send a test goal. The fleet manager sends `NavigateToPose`.

The image build runs `colcon build --packages-select r1_sim --symlink-install` and does not start Gazebo; starting the container launches `goal_sim_ground_truth.launch.py`, which loads the world.

```bash
cd src/r1_sim
docker/launch_docker.sh --build-image
```

Gazebo and RViz on this machine's display:

```bash
docker/launch_docker.sh --gui
```

On a machine that already has Jazzy:

```bash
colcon build --packages-select r1_sim --symlink-install
source install/setup.bash
ros2 launch r1_sim goal_sim_ground_truth.launch.py
```
