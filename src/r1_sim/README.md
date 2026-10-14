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

## Checking topics from the host

The container sets `GZ_PARTITION=r1` (override with `GZ_PARTITION=... docker/launch_docker.sh`). Gazebo tools on the host only see the container when they use the same partition; without it, gz-transport defaults to `hostname:username`, which differs between `root` in the container and your user on the host.

Run the check script, which sources ROS 2, restarts the ROS 2 daemon, and lists both sets of topics:

```bash
docker/check_topics.sh
```

Or by hand in your own shell:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export GZ_PARTITION=r1
ros2 daemon stop                 # a stale daemon shows only /parameter_events and /rosout
ros2 topic list --no-daemon --spin-time 3   # the default 0.5 s wait misses the container's nodes
gz topic -l
gz topic -e -t /model/robot_1/odometry -n 1
```

A container started before `GZ_PARTITION` was added uses `hostname:root`; for that one, use `export GZ_PARTITION="$(hostname):root"`.

Inside the container, `/ws/install/setup.bash` exists, so open a shell first and then source both files:

```bash
docker exec -it gazebo_sim bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
```

## Without Docker

On a machine that already has Jazzy:

```bash
colcon build --packages-select r1_sim --symlink-install
source install/setup.bash
ros2 launch r1_sim goal_sim_ground_truth.launch.py
```
