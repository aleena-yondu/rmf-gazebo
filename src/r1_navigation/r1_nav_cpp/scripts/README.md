# r1_nav_cpp scripts

## nav_order_stress_test.py

Full-system navigation load test: publishes a synthetic `PickTicket` on
`/teleop/order`, triggers the custom Nav2 BT with a dummy `NavigateToPose`
goal, and prints CPU/load plus filtered Nav2/BT/slam_toolbox warnings from
`/rosout` while each goal is active.

Prereq: nav stack running (`start_nav.sh`) so `/navigate_to_pose` is available.

```bash
source ~/yondu_ws/install/setup.bash
cd ~/yondu_ws/src/r1_navigation/r1_nav_cpp/scripts

# Interactive: prompts for a bin barcode (e.g. Y50-01-B-02) and one tote
python3 nav_order_stress_test.py

# Stress: cycle a 4x4 tote grid against a fixed bin, repeat twice
python3 nav_order_stress_test.py --cycle-totes --rows 4 --cols 4 --repeat 2 --clear-at-end
```

Common flags: `--bin` (bin barcode for cycle mode), `--pause` (seconds between
orders, default 2), `--goal-timeout` (cancel stuck goals, default 180 s),
`--clear-at-end` (publish `item_name=NONE` after the test), `--robot-name`,
`--pick-arm`. Ctrl-C stops cleanly.

Note: `ros2 run r1_nav_cpp nav_order_stress_test.py` only works after a
`colcon build` of `r1_nav_cpp` that postdates the script (the CMake install
rule GLOBs `scripts/` at configure time). Running via `python3` from `src/`
always works.
