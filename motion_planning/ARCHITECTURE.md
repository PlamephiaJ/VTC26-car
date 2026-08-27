# Motion-planning module guide

`RRT` is the ROS2 orchestration layer. Planning and control algorithms live in
small modules so they can be changed and tested without constructing a ROS
node. Public headers contain the detailed input, output, mutation, and boundary
contracts for every interface.

## Where to make a change

| Optimization target | Public interface | Implementation |
|---|---|---|
| RRT node callbacks, parameters, TF, publishers | `include/motion_planning/RRT.hpp` | `src/RRT.cpp` |
| RRT node/tree geometry, nearest, steer, near, reparent, path tracing | `include/motion_planning/rrt_tree.hpp` | `src/rrt_tree.cpp` |
| RRT* sampling and planning loop | `include/motion_planning/rrt_star_planner.hpp` | `src/rrt_star_planner.cpp` |
| Optimal trajectory arc length, progress projection, sampling, slicing | `include/motion_planning/optimal_trajectory.hpp` | `src/optimal_trajectory.cpp` |
| Optimal-reference/RRT-detour mode switching and safe rejoin | `include/motion_planning/reference_path_manager.hpp` | `src/reference_path_manager.cpp` |
| Local-path conversion/resampling, arc-length lookahead, Pure Pursuit, speed | `include/motion_planning/path_tracking.hpp` | `src/path_tracking.cpp` |
| LaserScan filtering plus static/live obstacle-layer lifetime | `include/motion_planning/dynamic_obstacle_map.hpp` | `src/dynamic_obstacle_map.cpp` |
| Occupancy coordinates, inflation, segment/polyline collision, root escape | `include/motion_planning/occupancy_grid.hpp` | `src/occupancy_grid.cpp` |
| Waypoint CSV input | `include/motion_planning/FileHandler.hpp` | `src/FileHandler.cpp` |
| RViz marker wrappers | `include/motion_planning/Visualization.hpp` | `src/Visualization.cpp` |

## One odometry-cycle data flow

1. `RRT::odom_callback()` records the current pose and refreshes TF.
2. `reference_path::Manager::update()` projects trajectory progress and creates
   the forward global goal plus local optimal reference.
3. If the optimal arc is clear, the local optimal reference is used directly.
4. If it is blocked, `rrt_star::Planner::plan()` builds a local detour to the
   same progress-based global goal.
5. `path_tracking` selects the short arc-length lookahead target and computes
   steering/speed on whichever local reference is active.
6. `RRT` publishes the path/tree markers and Ackermann command.

The optimal trajectory precomputes segment and cumulative arc lengths once.
After the first global projection, progress updates search only the configured
backward/forward arc window. A global projection is retried only when the local
projection is farther than `PROJECTION_FALLBACK_DISTANCE`.

While in RRT-detour mode, returning to the optimal reference requires all three
conditions in the same update:

- the optimal arc from current progress to the global goal is collision-free;
- distance from the vehicle to the optimal projection is no greater than
  `OPTIMAL_REJOIN_DISTANCE`;
- the direct short connector from the vehicle to that projection is clear.

Laser scans follow a separate short flow:

1. `dynamic_obstacles::valid_hit_points()` filters ranges and creates
   laser-frame hit points.
2. `RRT` applies TF because frame lookup is a ROS responsibility.
3. `RRT` retains sampled scans in a short rolling time window, rebuilds the
   dynamic layer at a configured fixed rate, and expires only old frames.

No planner subscribes to another vehicle's odometry. A moving vehicle is just
another obstacle observed by the current vehicle's own LiDAR. When RRT* cannot
find a detour, the node measures the arc distance to the first occupied point
on its blocked optimal reference and applies a proportional speed cap. It
therefore slows or stops using only its own perception. Returning from RRT mode
also requires all safe-rejoin conditions to remain continuously true for the
configured clearance time, which rejects one-frame obstacle dropouts.

## Interface contract convention

Each public declaration documents:

- **Input**: coordinate frame, units, required ranges, and ownership;
- **Return**: meaning of the value and failure representation;
- **Operation**: state or argument mutation, when applicable;
- **Boundary behavior**: invalid maps, empty paths, out-of-range coordinates,
  short paths, and rejected samples.

Algorithm modules do not publish ROS messages, query TF, or log. Those effects
remain in `RRT`, which makes module tests deterministic and keeps optimization
work localized.

## Running two vehicles

`launch/rrt.launch.py` accepts a `launch.vehicles` list in the selected YAML.
Set `launch.vehicle_mode` to `1` to launch only the first enabled vehicle, or
to `2` to launch the first two. It starts a separate `rrt_node_sim` process for
each selected car. The parameters under `rrt_node.ros__parameters` are shared
by all instances, while each vehicle's `ros__parameters` override its odometry,
scan, drive, dynamic-map, and control topics.

The shipped `config/rrt.yaml` connects the two default gym agents as follows:

| RRT namespace | Odometry | Laser scan | Drive command | Start/stop control |
|---|---|---|---|---|
| `/ego_racecar` | `/ego_racecar/odom` | `/scan` | `/drive` | `/ego_racecar/control` |
| `/opp_racecar` | `/opp_racecar/odom` | `/opp_scan` | `/opp_drive` | `/opp_racecar/control` |

Each vehicle entry also owns its `SPEED_STRAIGHT`, `SPEED_MEDIUM_TURN`, and
`SPEED_SHARP_TURN` values in metres per second. The shared low/medium steering
thresholds select between those three levels. Speed values must satisfy
`straight >= medium turn >= sharp turn > 0`.

Each vehicle also has an RGB `VISUALIZATION_PRIMARY_COLOR` for its path, goal,
and RRT branches, plus a `VISUALIZATION_ACCENT_COLOR` for its lookahead point
and RRT nodes. The default ego palette is blue/cyan and the opponent palette is
orange/magenta.

Both nodes share `/map`, but keep their dynamic maps and all relative
visualization topics inside their own namespaces. The node derives each
vehicle's TF frames from its namespace (`<namespace>/base_link` and
`<namespace>/laser`). The original single-car `launch.namespace` YAML format
is still supported when `launch.vehicles` is absent.

With the default safety setting (`start_on_launch: false`), the per-vehicle
topics still start or stop one car independently. The repository-level
`startrun.sh` and `stoprun.sh` publish one message to `/rrt/control`, which both
RRT nodes subscribe to, so the two commands share one publisher and DDS sample:

```bash
ros2 topic pub --once /ego_racecar/control std_msgs/msg/String "{data: start}"
ros2 topic pub --once /opp_racecar/control std_msgs/msg/String "{data: start}"
ros2 topic pub --once /ego_racecar/control std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /opp_racecar/control std_msgs/msg/String "{data: stop}"
./startrun.sh
./stoprun.sh
./startrun.sh ego
./stoprun.sh ego
./startrun.sh opp
./stoprun.sh opp
```

With no argument, each script controls all running vehicles through the shared
topic. The publisher waits until both RRT subscribers have been discovered
before sending the single shared command. Pass `ego`/`1` or `opp`/`2` to
control only that vehicle.
