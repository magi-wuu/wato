# WATonomous ASD Navigation

My implementation of the WATonomous ASD navigation assignment using C++ and ROS 2.

## Running

With Docker running:

```sh
./watod build
./watod up -d
```

Connect Foxglove to `ws://localhost:PORT` using the port printed by `watod`, then import:

```text
config/wato_asd_training_foxglove_config .json
```

Set the fixed frame to `sim_world` and display `/costmap`, `/map`, and `/path`.

To set a destination, use Foxglove's **Publish Point** tool on `/goal_point` with the frame `sim_world`.

After making code changes:

```sh
./watod build robot
./watod up -d robot
```

Stop everything with:

```sh
./watod down
```

## Navigation

The system is split into four ROS nodes:

* **Costmap:** converts lidar data into a local occupancy grid and inflates obstacles to give the robot enough clearance.
* **Map Memory:** transforms local observations into `sim_world` and combines them into a persistent global map.
* **Planner:** uses 8-connected A* to find a path from the robot to `/goal_point`.
* **Controller:** uses Pure Pursuit to follow `/path` and publishes movement commands to `/cmd_vel`.

## Design Choices

I allowed A* to travel through unknown space with an added cost. This lets the robot plan toward unexplored goals while still preferring areas it already knows are clear.

Obstacles are inflated because A* otherwise treats the robot like a point and can generate paths too close to walls for the actual robot to follow. Diagonal corner cutting is also blocked for the same reason.

I used Pure Pursuit so the robot follows a point farther along the path instead of trying to hit every grid cell individually, giving it smoother movement.

The navigation algorithms are kept in `*_core.cpp`, while `*_node.cpp` handles ROS communication. This made the algorithms easier to test without running the entire simulation.

## Testing

Had AI help me make some automated tests. I added them to check that the algorithms behaved correctly and to catch regressions while fixing issues. Tests cover the main costmap, mapping, A*, and Pure Pursuit logic, including obstacle inflation, map transformations, unreachable goals, diagonal corner blocking, and controller stopping/turning.
