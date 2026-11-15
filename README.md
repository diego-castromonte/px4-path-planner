# PX4 Path Planner

A UAV path-planning and trajectory-tracking project built with PX4, Gazebo, MAVSDK-Python, and Pygame.

The project plans a safe path through user-defined waypoints while avoiding grid obstacles and remaining inside a geofence. The resulting North/East route is streamed to PX4 in Gazebo, where the simulated vehicle tracks the trajectory with turn-aware speed control and yaw alignment.

## System Overview

```text
Pygame map and mission waypoints
        ↓
Geofence validation and A* path planning
        ↓
Bézier-based path smoothing
        ↓
10 Hz offboard position references with speed and yaw targets
        ↓
PX4 SITL + Gazebo simulation
        ↓
ULog analysis in PlotJuggler
```

## Current Features

- PX4 SITL running in WSL Ubuntu 24.04
- Gazebo for 3D vehicle simulation
- QGroundControl for 2D map and vehicle visualization
- MAVSDK-Python communication between the Python scripts and PX4
- Pygame grid-map interface with:
  - Start location
  - Three mission waypoints completed in order
  - Addable/removable black obstacle cells
  - A bounded geofence area
  - Real-time A* path visualization
- A* obstacle-avoidance route planning
- Conversion of planned grid routes into local North/East coordinates
- Bézier-based smoothing to reduce sharp 90-degree turns where space permits
- Continuous 10 Hz offboard position-target streaming instead of stopping at every route point
- Turn-aware speed profile:
  - Slows before tight turns
  - Accelerates toward cruise speed on clear straight sections
- Yaw commands that orient the vehicle along the route direction
- PX4 ULog analysis with PlotJuggler to compare reference and simulated position/velocity behavior

## Current Workflow

1. Run `geofence.py` and create a mission with a start point, obstacles, and three waypoints.
2. The planner validates the route and runs A* between each waypoint.
3. Copy the generated North/East route into `fly_astar_route.py`.
4. Start PX4 SITL with Gazebo in WSL.
5. Run the MAVSDK offboard-flight script.
6. Review the generated ULog in PlotJuggler.

## Project Structure

```text
px4_path_planner/
├── PathFinding/
│   ├── geofence.py           # Pygame map, geofence, obstacles, A* visualization
│   ├── astar.py              # A* prototype/work
│   ├── curve.py              # Path smoothing work
│   ├── fly_astar_route.py    # PX4 offboard route execution
│   ├── yaw.py                # Standalone yaw-control test
│   └── *.json                # Fixed test maps
├── connect_px4.py            # PX4 connection test
├── local_position.py         # Local-position telemetry test
├── takeoff.py                # Takeoff test
├── telemetry.py              # Telemetry test
├── mission.py                # Mission experiment
├── avoidance.py              # Avoidance experiment
└── multi_drone.py            # Multi-drone experiment
```

## Running the Simulation

Start PX4 SITL and Gazebo from WSL:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Activate the Python environment and run the flight script:
# Activate the MAVSDK environment
source ~/px4-python/bin/activate

# Run the route-execution script
python fly_astar_route.py

## Planned Improvements

- Reject or skip mission waypoints outside the geofence automatically
- Ramer-Douglas-Peucker (RDP) path simplification with collision checking
- Minimum-snap trajectory generation with time allocation
- Position, velocity, acceleration, yaw, and yaw-rate feed-forward references
- Fixed-seed random maps and randomized waypoint testing

## Notes

This project currently runs in PX4/Gazebo simulation. It is intended for safe trajectory-planning and tracking research before any physical-flight implementation.