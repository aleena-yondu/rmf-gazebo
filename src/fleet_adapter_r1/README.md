# fleet_adapter_r1

`fleet_adapter_r1` connects R1 picking robots to Open-RMF. It converts RMF plans into ROS requests and returns robot state to RMF.

| Direction | Interface | Data |
|---|---|---|
| Input | `/r1_fleet/robot_state` | `r1_interfaces/msg/FleetRobotState` |
| Output | `/r1_fleet/nav_request` | `r1_interfaces/msg/FleetNavRequest` |
| Output | `/r1_fleet/action_request` | `r1_interfaces/msg/FleetActionRequest` |
| Output | `/r1_fleet/mode_request` | `r1_interfaces/msg/FleetModeRequest` |
| Input | RMF | Navigation paths and `perform_action` requests |
| Output | RMF | Position, battery state, task progress, and completion |

The [central RMF container](https://github.com/YonduAI/rmf_core_server_docker) builds and starts this adapter. [`r1_fleet_client`](https://github.com/YonduAI/r1_fleet_client) implements the robot side of the topic protocol.

## How it fits

Open-RMF assigns paths and actions to this adapter. The adapter transforms RMF coordinates into robot coordinates.

The adapter supports these `perform_action` categories:

- `pick`
- `place`
- `approach_bin`

Zenoh carries `/r1_fleet/*` between the central ROS domain and each robot domain. The adapter waits for state from every configured robot before registration.

## Configuration

`config/config.yaml` contains the fleet and robot configuration:

| Section | Purpose |
|---|---|
| `rmf_fleet` | Fleet name, motion limits, battery model, profile, and task capabilities |
| `rmf_fleet.fleet_manager` | Topic names and nominal navigation speed |
| `reference_coordinates` | Matching points for the RMF-to-robot coordinate transform |
| `robots` | Robot names, starting map, charger, and update frequency |

The default robot is `brunson`. Its map name is `L1`.

`launch/fleet_adapter.launch.xml` uses `rmf_shared_maps/yondu/nav_graph.yaml` by default.

## Build and start

The central container is the normal deployment path:

```bash
cd ../rmf_dockerized
./build.sh
export SHIPHERO_API_KEY="YOUR_SHIPHERO_APP_TOKEN"
./run.sh
```

For a local ROS workspace, build and start only this adapter:

```bash
colcon build --packages-up-to fleet_adapter_r1
source install/setup.bash
ros2 launch fleet_adapter_r1 fleet_adapter.launch.xml
```

Override the configuration or navigation graph through the launch file:

```bash
ros2 launch fleet_adapter_r1 fleet_adapter.launch.xml \
  config_file:=/path/to/config.yaml \
  nav_graph:=/path/to/nav_graph.yaml
```

Before startup, make sure that these values agree:

- The robot name in `config.yaml` and `r1_fleet_client`
- The map name in robot state and the navigation graph
- The reference points for the RMF and robot frames
- The topic names on both sides of the Zenoh bridge
