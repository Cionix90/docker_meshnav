# docker_meshnav

The **`mesh_navigation`** stack provides a **complete 3D surface navigation system** for robots that move on **triangular mesh maps** — e.g., walking or tracked robots navigating uneven terrain, ramps, or stairs.

It extends the standard ROS 2 navigation ecosystem by integrating with **Move Base Flex (MBF)** and introducing a **layered mesh map representation**.

## Core Concept

Unlike traditional 2D costmaps used by `nav2`, the `mesh_navigation` stack operates on **2D manifolds embedded in 3D** — meaning the robot navigates *on* a 3D surface, not *through* a volumetric space.

At its heart, it consists of:

1. A **Mesh Map**: A 3D triangular mesh representation of the environment.  
2. A **Layered Map System**: A modular plugin-based cost structure that encodes traversability metrics.  
3. A **Move Base Flex–based navigation server** (`mbf_mesh_nav`) that handles:
   - Path planning (via mesh planners)
   - Path following/control (via mesh controllers)
   - Recovery behaviors
## 1. Framework & Submodule Overview  

### 1.1 What is MBF (Move Base Flex)  
The mesh_navigation stack is built on top of **MBF (Move Base Flex)**.  

- **MBF** is a flexible navigation framework (evolution of `move_base`) that supports loading multiple planner, controller, and recovery plugins at runtime.  
- It exposes action servers for planning, controlling, and recovery with rich feedback and result codes.  
- In `mesh_navigation`, MBF is used via the package `mbf_mesh_nav`, and core plugin interfaces are in `mbf_mesh_core`.  
- The architecture allows you to **extend** the system by creating your own planner, controller, or recovery plugin classes that adhere to MBF interfaces.  

### 1.2 mesh_navigation Submodules  
From the repository structure and documentation:

| Submodule | Description |
|------------|--------------|
| **mbf_mesh_core** | Contains plugin interfaces (`MeshPlanner`, `MeshController`, `MeshRecovery`) derived from MBF abstract interfaces. |
| **mbf_mesh_nav** | The actual navigation server for mesh navigation built for MBF using the layered mesh map. |
| **mesh_map** | Implementation of the mesh map representation (triangular mesh) and the layered-map abstraction. |
| **mesh_layers** | A set of plugin layers that compute terrain / traversability metrics on top of the mesh (HeightDiff, Roughness, Steepness, etc.). |
| **dijkstra_mesh_planner**, **cvp_mesh_planner** | Example planner plugins for path planning on the mesh. |

### 1.3 How to Expand / Extend the Framework  
You can extend mesh_navigation by:  

- **Adding your own planner plugin**: derive from `mbf_mesh_core::MeshPlanner` and register via pluginlib.  
- **Adding your own controller plugin**: derive from `mbf_mesh_core::MeshController`.  
- **Adding your own recovery plugin**: derive from `mbf_mesh_core::MeshRecovery`.  
- **Adding your own mesh layer plugin**: implement a new layer in `mesh_layers`, register via pluginlib, and configure in your mesh map.  
- **Combining layers differently**: select and tune layers to match your robot’s terrain handling capabilities.  

---

## 2. Available Mesh Layers & Their Purpose  

| Layer Plugin | Type Specifier | Description |
|---------------|----------------|-------------|
| **HeightDiffLayer** | `mesh_layers/HeightDiffLayer` | Computes local height differences around vertices → penalizes sudden elevation changes. |
| **RoughnessLayer** | `mesh_layers/RoughnessLayer` | Measures fluctuation of surface normals → penalizes rough terrain. |
| **SteepnessLayer** | `mesh_layers/SteepnessLayer` | Computes arccos of the mesh-normal z-component → represents slope cost. |
| **RidgeLayer** | `mesh_layers/RidgeLayer` | Identifies ridge lines where slope changes abruptly. |
| **ClearanceLayer** | `mesh_layers/ClearanceLayer` | Computes clearance radius for robot footprint above mesh / obstacles beneath surface. |
| **InflationLayer** | `mesh_layers/InflationLayer` | Inflates cost around “lethal” or high-cost regions (similar to costmaps). |
| **BorderLayer** | `mesh_layers/BorderLayer` | Maintains borders/edges of the mesh or map boundary. |
| **ObstacleLayer** | `mesh_layers/ObstacleLayer` | Identifies obstacles on mesh surface (e.g., from sensor or map annotation). |

---

## 3. Key Configuration Parameters for Layers & Plugins  

### 3.1 Mesh Map / Layer General Settings  

| Parameter | Description |
|------------|-------------|
| `mesh_map.layers` | List of layer plugin names (e.g., `["mesh_layers/HeightDiffLayer", "mesh_layers/ClearanceLayer"]`). |
| `mesh_map.layer_settings.<layer_name>.radius` | Radius around each mesh vertex used by the layer to compute metrics (in meters). |
| `mesh_map.layer_settings.<layer_name>.cost_scale` | Scaling factor to convert computed metric into navigation cost. |
| `mesh_map.robot_height` | Robot height above the mesh surface (used for clearance computations). |
| `mesh_map.robot_max_slope` | Maximum slope the robot can traverse (used in steepness and clearance layers). |
| `mesh_map.robot_radius` | Robot footprint radius on the mesh for clearance/inflation layers. |

---

### 3.2 Planner / Controller Plugin Parameters  

| Parameter | Description |
|------------|-------------|
| `mesh_planner.type` | Plugin type for planner, e.g., `cvp_mesh_planner/CVPMeshPlanner` or `dijkstra_mesh_planner/DijkstraMeshPlanner`. |
| `mesh_controller.type` | Plugin type for controller, e.g., `mesh_controller/MeshController`. |
| `mesh_planner.max_edge_length` | Maximum edge length in mesh to be considered for planning (meters). |
| `mesh_planner.cost_weight_smoothness` | Weight balancing smoothness vs. path length. |
| `mesh_controller.max_velocity` | Maximum allowed velocity on the mesh surface (m/s). |
| `mesh_controller.max_acceleration` | Maximum allowed acceleration (m/s²). |

---

### 3.3 Layer-Specific Example Parameters  

#### HeightDiffLayer  
- `height_diff_layer.radius` — Neighborhood radius (m).  
- `height_diff_layer.max_height_diff` — Maximum height difference considered (m).  
- `height_diff_layer.cost_scale` — Scaling factor to convert metric to cost.  

#### RoughnessLayer  
- `roughness_layer.radius` — Neighborhood radius (m).  
- `roughness_layer.max_normal_variation` — Threshold for surface normal variation.  
- `roughness_layer.cost_scale` — Scaling factor for roughness cost.  

#### SteepnessLayer  
- `steepness_layer.max_slope_deg` — Max slope in degrees robot can handle.  
- `steepness_layer.cost_scale` — Scaling factor.  

#### ClearanceLayer  
- `clearance_layer.robot_height` — Robot height (m).  
- `clearance_layer.max_clearance_violation` — Allowed clearance violation (m).  
- `clearance_layer.cost_scale` — Cost scaling factor.  

#### InflationLayer  
- `inflation_layer.inflation_radius` — Radius around high-cost vertices (m).  
- `inflation_layer.cost_scaling_factor` — How quickly cost decays with distance.  

#### ObstacleLayer  
- `obstacle_layer.lethal_distance` — Distance threshold for lethal obstacles (m).  
- `obstacle_layer.enabled_topics` — Sensor topics or map sources providing obstacle data.  

---

### 3.4 MBF / Navigation Server Parameters  

| Parameter | Description |
|------------|-------------|
| `use_sim_time` | Whether to use `/clock` simulation time (typical for simulation). |
| `global_frame` | Frame id for global planning (e.g., `map`). |
| `robot_base_frame` | Robot frame (e.g., `base_link`). |
| `goal_tolerance_xy` | Tolerance in XY for goal acceptance (m). |
| `goal_tolerance_yaw` | Yaw tolerance for goal acceptance (rad). |

---

## 4.  Container Start Up
the containers can be started using the command:
``` docker compose up ```
or 
``` docker compose up -d``` 
to start the container in a deferred way.

To open a bash session in a specific container uses:

``` docker compose exec <container-name> bash ```

To start the navigation framework based on the provided  world:

``` ros2 launch mesh_navigation_tutorials mesh_navigation_tutorials_launch.py world_name:=<world-name>```

the available world are: 
- tray
- floor_is_lava
- parking_garage


