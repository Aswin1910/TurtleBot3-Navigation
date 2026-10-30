# TurtleBot3 + Gazebo Harmonic + Nav2 (ROS 2 Jazzy)

Task:
1. Set up TurtleBot3 in a Gazebo environment of your choice.
2. Configure Nav2 so it drives autonomously to a goal while avoiding obstacles.

This workspace contains one ROS 2 package, **`tb3_nav2_sim`**, with:

| Path | What it is |
|---|---|
| `worlds/obstacle_course.sdf.xacro` | Custom Gazebo world: a 10 × 10 m walled arena with two partitions (an S-shaped route), crates, pillars and **2 red "surprise" obstacles** |
| `maps/obstacle_course.{pgm,yaml}` | Occupancy map for Nav2. It does **not** contain the red obstacles, so the robot only finds them with its LiDAR |
| `params/nav2_params.yaml` | Nav2 config (AMCL, NavFn planner, MPPI controller, costmaps), based on the Jazzy defaults. Lines marked `# CHANGED` show the tuning |
| `launch/sim_nav.launch.py` | Starts Gazebo, spawns TurtleBot3 Waffle, and starts Nav2 + RViz |
| `scripts/send_goal.py` | Sends a navigation goal from Python (`nav2_simple_commander`) |
| `tools/generate_world_and_map.py` | Builds the world and the map from **one** obstacle list, so they always match |

```
Robotics/
├── setup.sh
├── README.md
└── tb3_nav_ws/src/tb3_nav2_sim/...
```

---

## 0. Requirements
- Ubuntu 24.04 with **ROS 2 Jazzy** (you're on Windows, so this means WSL2 with Ubuntu 24.04; WSLg shows the GUIs)
- Gazebo **Harmonic** (`gz sim`), which is the Gazebo version paired with Jazzy

Inside WSL, this Windows folder is at `/mnt/c/Users/aswin/Documents/Robotics`.

## 1. Install and build (one time)
```bash
cd /mnt/c/Users/aswin/Documents/Robotics
bash setup.sh
```
`setup.sh` installs `navigation2`, `nav2_bringup`, `nav2_minimal_tb3_sim` (the TurtleBot3 Waffle model for Gazebo Harmonic, maintained by the Nav2 team), `nav2_simple_commander` and `ros_gz`, then runs `colcon build`.

> Tip: building under `/mnt/c/...` works but is slow. For better speed, copy `tb3_nav_ws` into your Linux home (`cp -r tb3_nav_ws ~/`) and build it there.

## 2. Launch the simulation and Nav2
**Terminal 1**
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/aswin/Documents/Robotics/tb3_nav_ws/install/setup.bash
ros2 launch tb3_nav2_sim sim_nav.launch.py
```
Gazebo opens with the TurtleBot3 Waffle at the bottom-left corner (-4, -4), and RViz opens with the map. The robot's start pose is already set (in the AMCL params), so it is localised right away. You should see the laser scan line up with the map walls.

## 3. Send a goal
**Option A: RViz.** Click **Nav2 Goal** on the toolbar, then click and drag on the map, for example near the top-right corner.

**Option B: Python script.** In terminal 2:
```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/aswin/Documents/Robotics/tb3_nav_ws/install/setup.bash
ros2 run tb3_nav2_sim send_goal.py            # default goal (4, 4): top-right corner
ros2 run tb3_nav2_sim send_goal.py 3.5 -3 90  # x y yaw(deg)
```

**Option C: CLI.**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 4.0}, orientation: {w: 1.0}}}}" --feedback
```

### What you should see
1. The global planner (NavFn) draws a path through the S-shaped route: right along the bottom, up through the right gap, left across the middle, up through the left gap, then right to the goal.
2. In the middle corridor, the **red box** is not on the map. When the LiDAR sees it, it appears in the local and global costmaps (the obstacle layer), and MPPI plus the replanner (every 1 s) steer around it.
3. The same happens with the **red barrel** in the top corridor.
4. The robot reaches the goal and the script prints `Goal reached!`.

## 4. How Nav2 is configured (summary)
- **Localisation:** AMCL with `set_initial_pose: true` at (-4, -4, 0), which matches the spawn pose. `laser_max_range: 3.5` matches the TurtleBot3 LDS.
- **Global costmap:** static layer (map), plus obstacle layer (`/scan`), plus inflation. Unmapped obstacles get added, so the global path is replanned around them.
- **Local costmap:** a 4 × 4 m rolling window with a voxel layer from `/scan`, plus inflation.
- **Inflation:** `inflation_radius 0.55`, `cost_scaling_factor 4.0`. This keeps the robot (radius 0.22 m) clear of walls but still lets it fit through the gaps.
- **Planner:** NavFn (Dijkstra). **Controller:** MPPI. **Behaviour tree:** replans every 1 s, with spin, back-up and wait recoveries.

## 5. Changing the environment
Edit the `BOXES` / `CYLINDERS` lists in `tools/generate_world_and_map.py`, then:
```bash
cd tb3_nav_ws/src/tb3_nav2_sim && python3 tools/generate_world_and_map.py
cd ../.. && colcon build --symlink-install
```
Set `in_map=False` to make an obstacle the robot has to discover with its LiDAR.

Other launch options:
```bash
ros2 launch tb3_nav2_sim sim_nav.launch.py headless:=True   # no Gazebo GUI (faster)
ros2 launch tb3_nav2_sim sim_nav.launch.py slam:=True       # build the map live with slam_toolbox instead of loading one
```
(`slam:=True` needs `sudo apt install ros-jazzy-slam-toolbox`.)

## 6. Troubleshooting
| Symptom | Fix |
|---|---|
| No `/scan` or a black Gazebo window in WSL | `export LIBGL_ALWAYS_SOFTWARE=1` before launching, or update your GPU driver / WSL (`wsl --update`) |
| `Package 'nav2_minimal_tb3_sim' not found` | `sudo apt install ros-jazzy-nav2-minimal-tb3-sim` |
| Robot not localised (scan doesn't match the map) | In RViz, use **2D Pose Estimate** at the robot's position, or run `send_goal.py`, which sets the initial pose |
| `$'\r': command not found` | Files got Windows line endings. `setup.sh` fixes this automatically, or run `sed -i 's/\r$//' setup.sh` |
| Robot gets stuck in a gap | Lower `inflation_radius` in `params/nav2_params.yaml` (for example to 0.45) and rebuild |
