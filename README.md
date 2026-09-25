# WATonomous ASD navigation assignment

C++17 / ROS 2 Humble implementation of the [ASD assignment](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/).

## Run

Start Docker Desktop (macOS/Windows) or Docker Engine (Linux), then from this folder:

```sh
./watod build
./watod up -d
./watod ps
./watod logs robot
```

The configured modules are `robot gazebo vis_tools`. The bridge port is printed by
watod and is also in `modules/.env` as `FOXGLOVE_BRIDGE_PORT`. Open Foxglove,
choose a Foxglove WebSocket connection (`ws://localhost:PORT`), then import
`config/wato_asd_training_foxglove_config .json` (the starter filename contains a space).
Set the 3D fixed frame to `sim_world`. Display `/costmap`, `/map`, `/path`, and the robot.
Use the 3D panel's publish-point tool with topic `/goal_point` and frame `sim_world`.
Choose a goal inside the walls, away from obstacles. The robot replans as it discovers obstacles.

A terminal goal can also be sent inside the robot container:

```sh
./watod exec robot bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: sim_world}, point: {x: 5.0, y: 0.0, z: 0.0}}"'
```

Stop with `./watod down`. Rebuild changes with `./watod build robot`, then
`./watod up -d robot`. Avoid using teleoperation while a navigation goal is active,
as both publish velocity commands.

## Design

| Package | Input | Output | Algorithm |
| --- | --- | --- | --- |
| costmap | `/lidar` | `/costmap` | Ray clearing, obstacle marking, linear inflation |
| map_memory | `/costmap`, `/odom/filtered`, TF | `/map` | Timestamped local-to-world transform, persistent grid fusion |
| planner | `/map`, `/goal_point`, `/odom/filtered` | `/path` | Cost-aware 8-connected A*, goal state and periodic replanning |
| control | `/path`, `/odom/filtered` | `/cmd_vel` | Pure pursuit, turn-in-place, goal stop and freshness watchdogs |

`*_core.cpp` files contain algorithms; `*_node.cpp` files handle ROS transport and state.
`navigation_common` holds shared grid geometry. Each package's `config/params.yaml`
contains its tuning parameters. The global frame is `sim_world`; the local costmap
uses the laser scan frame. The starter odometry represents the lidar pose, as specified
by the assignment. Mapping uses TF at the scan timestamp to avoid motion smearing.

The local grid is 0.1 m/cell and the global grid 0.2 m/cell. Only observed rays clear
space; unknown cells never erase old observations. When several local cells map to one
global cell, their maximum new cost wins. The map publishes on startup with transient-local
QoS so a late planner receives it. Updates are throttled to 5 Hz, gated by translation
or rotation, with a one-second refresh for newly visible obstacles while stationary.
The wiki gives inconsistent 1.5 m / 5 m update examples; the default is a configurable
0.3 m for this large robot and conservative navigation speed.

A* permits unknown space with a penalty so exploration can begin. It blocks costs >=40,
uses a Euclidean heuristic, and forbids diagonal corner cutting. If the starting pose
is already inside an inflated margin, it allows escape through non-increasing costs
until reaching free space; occupied cells remain blocked. A 3 m linear inflation
radius gives approximately 1.8 m clearance at that threshold, accounting conservatively
for the simulator's large chassis and lidar offset. Paths are replaced on map updates
and at least every second; an invalid/unreachable goal publishes an empty path. Goals
must use `sim_world` and lie within the fixed 60 m global grid. The controller stops on
empty paths, stale data, frame mismatch, or goal arrival. Goal timeout is 180 simulation
seconds. Both planner and controller use a 0.3 m goal tolerance.

Simulation time is bridged on `/clock` and enabled in the robot launch file. The control
receipt watchdog uses a steady wall clock as well. The starter odometry's previous-transform
flag is initialized before use.

## Tests

Build and run the core tests in a ROS container, preserving source and build results:

```sh
docker run --rm -v "$PWD/src/robot:/workspace/src:ro" \
  --entrypoint bash ghcr.io/watonomous/wato_asd_training/robot:main -lc \
  'source /opt/ros/humble/setup.bash; cd /workspace; colcon build --cmake-args -DBUILD_TESTING=ON; colcon test --return-code-on-test-failure; colcon test-result --verbose'
```

Tests cover laser invalid values and infinity, linear inflation, unknown retention,
rotation/translation, many-to-one fusion, A* detours and failure cases, diagonal corner
blocking, same-cell goals, and pure-pursuit stopping/turning limits.

The isolated ROS integration test starts the actual four C++ nodes with synthetic
laser, odometry, and TF messages. It does not restart or interact with Gazebo:

```sh
docker run --rm -e ROS_DOMAIN_ID=81 -v "$PWD/tests:/tests:ro" \
  --entrypoint bash ghcr.io/watonomous/wato_asd_training/robot:main -lc \
  'source /opt/ros/humble/setup.bash; source /opt/watonomous/setup.bash; python3 /tests/ros_smoke_test.py'
```

The robot Dockerfile also runs the core tests automatically before producing an image.
The simulator compose file mounts the launch directory at its installed package path,
so clock-bridge changes take effect after recreating the simulator container.

## Submission

Record Foxglove showing the robot navigating to several goals around obstacles with the
map and path visible. Push this folder to your own GitHub repository and submit that
link plus the demonstration video in the assignment-completions Discord channel.
Credit collaborators and AI assistance according to the assignment rules. This implementation
was created with Codex assistance; review the algorithms so you can explain and tune them.
