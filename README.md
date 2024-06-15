## A. Overview
This is the group project of the course Multi Discplinary Project of Master Robotics at TU Delft. The task is to cover a field while collecting "manure" and avoiding other fake "cows" and robots.

![robot](./robot.png) 
![screenshot](./screenshot.png)

- `onboard_tool_ws` is for onboard image compress and teleop node. Other packages are all run on PC. 
- `perception` folder 
    - image detection based on `yolov8` and `yolov8_ros` https://github.com/qq44642754a/Yolov8_ros
    - image depth estimation based on flat ground assumption in modified `yolov8_ros`
    - 2D `obstacle_tracker` based on static obstacle assumption. Obstacles are appended to tracking list after 2 frames of observation, and removed after 3 lost frames. Association is judged based on distance and label.
    - In `obstacle_tracker`, Pointcloud2 circles with radius of 0.1 or 0.15 m are generated for detected obstables, in case they are not discoverage by LiDAR. 2D point cloud is used for local planning. 
    - Real-time detection result, bird-eye view of base_link is visualized.
- `state-estimation` folder
    - `gmapping`
    - `amcl`
    - navigation stack's `move_base` package that can subscribe to point cloud 2D and output cmd/vel
    - `costmap_converter` to convert the costmap to polygons to be used by other local planners like teb-planner
    - `costmap_2d` to generate the standalone costmap to be used by other packages like `costmap_converter`
    - `lidar_preprocess` filter the laser scan by angles and distance to remove the points reflected by the robot itself. The result is published in `pointcloud2`.
- `path_planning` folder.
The `path_planning/swarm-functions` modify the cpswarm repo https://github.com/cpswarm/swarm_functions to achieve map division and global coverage path planning with path width = 0.5 * map resolution.
    - area_division, seperate the map to two, and publish the starting position and the divided map for each robot.
    - coverage_path, generate very basic coverage path for each robot.
- `HRI` folder.
Not implemented web interface for remote monitor and control
- `tool` folder.
Scripts for standalone tests of some node's input and output 


## B. Dependencies
First run this command to install all ros dependencies
```
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
```
Then in the python environment:
```
pip install torch # or other commands to install pytorch
pip install ultralytics # for yolov8, in the future may change
```
To run this package, one must create a mirte workspace in which you clone this repository inside of the `src` folder.
```
catkin_make
source devel/setup.bash
```
## C. Perception and State Estimation

#### 1. Mapping
First map the environment by 
```
roslaunch gmapping slam_gmapping_pr2.launch
```
After finish mapping, run
```
rosrun map_server map_saver -f the_name_of_map
```
Then copy the `the_name_of_map.yaml` and `the_name_of_map.pgm` to the `state-estimation/amcl/map/`
And modify  the `map_file` in the `state-estimation/amcl/launch/amcl_mirte.launch` to this `the_name_of_map.yaml`.

#### 2. Detection and Localization
To launch the perception as a whole, 
- First download the weight for yolov8-m on PC:
```
cd perception/yolov8_ros/weights
```
And download the weight from https://drive.google.com/file/d/1CxDn0tjxvM2x7RfnPRE_kZnSqmfxa16j/view?usp=sharing. The model is trained on `perception/yolov8_ros/weights/yolo-manure.ipynb`
- on robot:
```
export ROS_IP=$(hostname -I)
cd ~/onboard_tool_ws && source devel/setup.bash && roslaunch image_compress image_compress.launch
```
- on laptop
```
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://192.168.xx.xx:11311
roslaunch yolov8_ros perception_all.launch
```
Then you should see three windows: rviz for amcl and costmap, yolo detection result, and BEV tracking of cows and manure

## D. Path Planning and Control

### 1. Teleop with Obstacle Avoidance
Start the move_base with global and local planner, in rviz give goal pose to it. Problem: too far and too fast motion with this low localization frequency leads to unstability. Need to improve the localization frequency.
```
roslaunch move_base move_base.launch
```

### 2. Global Coverage Path planning
- First change the `delta` parameter in the `slam_gmapping_pr2.launch` to create a low resolution occupancy grid map
- Copy the generated `.pgm` and `.yaml` file to the `state-estimation/amcl/map/`, and modify the `map_file` in the `path_planning/swarm-functions/area_division/launch/area_path.launch` to the name of the yaml file.
- Then set the starting point at **line 63** of `path_planning/swarm-functions/area_division/src/main.cpp`. The starting point must be a free cell in the grid. The unit is pixel. Use the GIMP to find the suitable coordinate manually. The origin is the bottom-left corner of the image.
- Then manually set the offset of the generated path. Without this step, the path generation thought the origin is the bottom-left corner of the map, while the amcl use the origin offset in the yaml to set the starting point of gmapping as the origin point. In **line 106** of `path_planning/swarm-functions/coverage_path/src/coverage_path.cpp` in `generate_path` function, set the offset according to the origin x and y in the yaml file of the low resolution map.
- Finally run
```
roslaunch area_division area_path.launch
```


### 3. Follow the Global Path
In one terminal:
```
roslaunch move_base move_base.launch
```
In another terminal:
```
cd src/state-estimation/move_base/src && python coverage_path_convert.py
```
This script will convert the global path with poses to the move_base as its goal by distance. Still need test because it does not consider the orientation difference.

## E. Interface with Future Development
- Detection and tracking result is published on topic `yolo_detections_map` in `PointDetectionArray` in `map` frame. The msg is a custom msg defined in `yolov8_ros_msgs`. It is an array of `PointDetection` which consists of a `geometry_msgs/Point`, a `string` and a `float`.
- The costmap has information of obstacles detected by both LiDAR and Camera, and is published on topic `/costmap_node/costmap/costmap` in `map` frame in  `nav_msgs/OccupancyGrid`.
- The costmap is also converted to polygon by `costmap_converter` package. It is published on topic `/standalone_converter/costmap_obstacles` in `map` frame in `costmap_converter/ObstacleArrayMsg`
- In `obstacle_tracker.py` the function `signal_decision` can enable beap or the LED strip (not yet implemented), or send stop command to controller (not yet implemented) given the number and distance of each kind of obstacle detected by camera.

## F. File Structure
```
.
├── HRI
├── onboard_tools_ws
│   ├── record_all.sh
│   ├── record_lidar_odom.sh
│   └── src
│       ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│       ├── image_compress
│       │   ├── CMakeLists.txt
│       │   ├── launch
│       │   │   └── image_compress.launch
│       │   ├── package.xml
│       │   └── src
│       │       ├── clock_server.py
│       │       └── compress_convert.py
│       └── manual_control
│           └── teleop_slow
│               ├── CMakeLists.txt
│               ├── package.xml
│               └── teleop_slow.launch
├── path_planning
│   └── swarm-functions
│       ├── area_division
│       │   ├── CMakeLists.txt
│       │   ├── include
│       │   │   ├── area_division.h
│       │   │   └── lib
│       │   │       ├── area_division.h
│       │   │       └── connected_components.h
│       │   ├── launch
│       │   │   ├── area_division.launch
│       │   │   └── area_path.launch
│       │   ├── package.xml
│       │   ├── param
│       │   │   └── area_division.yaml
│       │   ├── README.md
│       │   └── src
│       │       ├── area_division.cpp
│       │       ├── lib
│       │       │   ├── area_division.cpp
│       │       │   └── connected_components.cpp
│       │       └── main.cpp
│       └── coverage_path
│           ├── CHANGELOG.rst
│           ├── CMakeLists.txt
│           ├── include
│           │   ├── coverage_path.h
│           │   └── lib
│           │       ├── edge.h
│           │       ├── mst_path.h
│           │       └── spanning_tree.h
│           ├── launch
│           │   └── coverage_path.launch
│           ├── package.xml
│           ├── param
│           │   └── coverage_path.yaml
│           ├── README.md
│           ├── src
│           │   ├── coverage_path.cpp
│           │   └── lib
│           │       ├── edge.cpp
│           │       ├── mst_path.cpp
│           │       └── spanning_tree.cpp
│           └── test
│               ├── coverage_path.yaml
│               ├── ma.yaml
│               ├── rosconsole.config
│               ├── test_coverage_path.cpp
│               ├── test_coverage_path.test
│               ├── test_edge.cpp
│               ├── test_mst_path.cpp
│               └── test_spanning_tree.cpp
├── perception
│   ├── obstacle_tracker
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── obstacle_tracker.launch
│   │   ├── package.xml
│   │   └── src
│   │       └── obstacle_tracker.py
│   ├── README_CN.md
│   ├── README.md
│   ├── yolov8_ros
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── perception_all.launch
│   │   │   └── yolo_v8.launch
│   │   ├── package.xml
│   │   ├── src
│   │   │   ├── compress_convert.py
│   │   │   ├── transformation_convert.py
│   │   │   └── yolo_v8.py
│   │   └── weights
│   │       ├── cow-2-320-m.pt
│   │       └── yolo-manure.ipynb
│   └── yolov8_ros_msgs
│       ├── CMakeLists.txt
│       ├── msg
│       │   ├── BoundingBoxes.msg
│       │   ├── BoundingBox.msg
│       │   ├── PointDetectionArray.msg
│       │   └── PointDetection.msg
│       └── package.xml
├── README.md
├── robot.png
├── screenshot.png
├── state-estimation
│   ├── amcl
│   │   ├── cfg
│   │   │   └── AMCL.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── examples
│   │   │   ├── amcl_diff.launch
│   │   │   └── amcl_omni.launch
│   │   ├── include
│   │   │   └── amcl
│   │   │       ├── map
│   │   │       │   └── map.h
│   │   │       ├── pf
│   │   │       │   ├── eig3.h
│   │   │       │   ├── pf.h
│   │   │       │   ├── pf_kdtree.h
│   │   │       │   ├── pf_pdf.h
│   │   │       │   └── pf_vector.h
│   │   │       └── sensors
│   │   │           ├── amcl_laser.h
│   │   │           ├── amcl_odom.h
│   │   │           └── amcl_sensor.h
│   │   ├── launch
│   │   │   ├── amcl_mirte.launch
│   │   │   └── rivz.rviz
│   │   ├── map
│   │   │   ├── 1984.pgm
│   │   │   ├── 20.pgm
│   │   │   ├── 20.yaml
│   │   │   ├── 30cm_res_barn_2.pgm
│   │   │   ├── 30cm_res_barn_2.yaml
│   │   │   ├── 5cm_res_barn_1.pgm
│   │   │   ├── 5cm_res_barn_1.yaml
│   │   │   ├── 5cm_res_barn_2.pgm
│   │   │   ├── 5cm_res_barn_2.yaml
│   │   │   ├── 5cm_res_small_barn.pgm
│   │   │   ├── 5cm_res_small_barn.yaml
│   │   │   ├── 5m_10cm_res_cropped.pgm
│   │   │   ├── 5m_10cm_res_cropped.yaml
│   │   │   ├── 5m_10cm_res.pgm
│   │   │   ├── 5m_10cm_res.yaml
│   │   │   ├── 5m_high_res.pgm
│   │   │   ├── 5m_high_res.yaml
│   │   │   ├── 60cm_res_barn_2.pgm
│   │   │   ├── 60cm_res_barn_2.yaml
│   │   │   ├── 60cm_res_small_barn.pgm
│   │   │   ├── 60cm_res_small_barn.yaml
│   │   │   ├── barn_zoo.pgm
│   │   │   ├── barn_zoo.yaml
│   │   │   ├── down_sample.py
│   │   │   ├── gourd_barn_lowres.pgm
│   │   │   ├── gourd_barn_lowres.yaml
│   │   │   ├── gourd_barn_midres.pgm
│   │   │   ├── gourd_barn_midres.yaml
│   │   │   ├── gourd_barn.pgm
│   │   │   ├── gourd_barn.yaml
│   │   │   ├── test_barn.pgm
│   │   │   ├── test_barn.png
│   │   │   ├── test_barn.yaml
│   │   │   ├── white_ground.pgm
│   │   │   ├── white_ground.yaml
│   │   │   ├── zusterlaan.pgm
│   │   │   └── zusterlaan.yaml
│   │   ├── package.xml
│   │   ├── src
│   │   │   ├── amcl
│   │   │   │   ├── map
│   │   │   │   │   ├── map.c
│   │   │   │   │   ├── map_cspace.cpp
│   │   │   │   │   ├── map_draw.c
│   │   │   │   │   ├── map_range.c
│   │   │   │   │   └── map_store.c
│   │   │   │   ├── pf
│   │   │   │   │   ├── eig3.c
│   │   │   │   │   ├── pf.c
│   │   │   │   │   ├── pf_draw.c
│   │   │   │   │   ├── pf_kdtree.c
│   │   │   │   │   ├── pf_pdf.c
│   │   │   │   │   └── pf_vector.c
│   │   │   │   └── sensors
│   │   │   │       ├── amcl_laser.cpp
│   │   │   │       ├── amcl_odom.cpp
│   │   │   │       └── amcl_sensor.cpp
│   │   │   ├── amcl_node.cpp
│   │   │   └── include
│   │   │       └── portable_utils.hpp
│   │   └── test
│   │       ├── basic_localization.py
│   │       ├── basic_localization_stage.xml
│   │       ├── global_localization_stage.xml
│   │       ├── rosie_multilaser.xml
│   │       ├── set_initial_pose_delayed.xml
│   │       ├── set_initial_pose.xml
│   │       ├── set_pose.py
│   │       ├── small_loop_crazy_driving_prg_corrected.xml
│   │       ├── small_loop_crazy_driving_prg.xml
│   │       ├── small_loop_prf.xml
│   │       ├── texas_greenroom_loop_corrected.xml
│   │       ├── texas_greenroom_loop.xml
│   │       ├── texas_willow_hallway_loop_corrected.xml
│   │       └── texas_willow_hallway_loop.xml
│   ├── costmap_2d
│   │   ├── cfg
│   │   │   ├── Costmap2D.cfg
│   │   │   ├── GenericPlugin.cfg
│   │   │   ├── InflationPlugin.cfg
│   │   │   ├── ObstaclePlugin.cfg
│   │   │   └── VoxelPlugin.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── costmap_plugins.xml
│   │   ├── include
│   │   │   └── costmap_2d
│   │   │       ├── array_parser.h
│   │   │       ├── costmap_2d.h
│   │   │       ├── costmap_2d_publisher.h
│   │   │       ├── costmap_2d_ros.h
│   │   │       ├── costmap_layer.h
│   │   │       ├── costmap_math.h
│   │   │       ├── cost_values.h
│   │   │       ├── footprint.h
│   │   │       ├── inflation_layer.h
│   │   │       ├── layered_costmap.h
│   │   │       ├── layer.h
│   │   │       ├── observation_buffer.h
│   │   │       ├── observation.h
│   │   │       ├── obstacle_layer.h
│   │   │       ├── static_layer.h
│   │   │       ├── testing_helper.h
│   │   │       └── voxel_layer.h
│   │   ├── launch
│   │   │   ├── bak
│   │   │   ├── example.launch
│   │   │   ├── example_params.yaml
│   │   │   └── example_params.yaml-bak
│   │   ├── msg
│   │   │   └── VoxelGrid.msg
│   │   ├── package.xml
│   │   ├── plugins
│   │   │   ├── inflation_layer.cpp
│   │   │   ├── obstacle_layer.cpp
│   │   │   ├── static_layer.cpp
│   │   │   └── voxel_layer.cpp
│   │   ├── src
│   │   │   ├── array_parser.cpp
│   │   │   ├── costmap_2d_cloud.cpp
│   │   │   ├── costmap_2d.cpp
│   │   │   ├── costmap_2d_markers.cpp
│   │   │   ├── costmap_2d_node.cpp
│   │   │   ├── costmap_2d_publisher.cpp
│   │   │   ├── costmap_2d_ros.cpp
│   │   │   ├── costmap_layer.cpp
│   │   │   ├── costmap_math.cpp
│   │   │   ├── footprint.cpp
│   │   │   ├── layer.cpp
│   │   │   ├── layered_costmap.cpp
│   │   │   └── observation_buffer.cpp
│   │   └── test
│   │       ├── array_parser_test.cpp
│   │       ├── coordinates_test.cpp
│   │       ├── costmap_params.yaml
│   │       ├── costmap_tester.cpp
│   │       ├── footprint_tests.cpp
│   │       ├── footprint_tests.launch
│   │       ├── inflation_tests.cpp
│   │       ├── inflation_tests.launch
│   │       ├── module_tests.cpp
│   │       ├── obstacle_tests.cpp
│   │       ├── obstacle_tests.launch
│   │       ├── simple_driving_test.xml
│   │       ├── static_tests.cpp
│   │       ├── static_tests.launch
│   │       ├── TenByTen.pgm
│   │       └── TenByTen.yaml
│   ├── costmap_converter
│   │   ├── cfg
│   │   │   └── dynamic_reconfigure
│   │   │       ├── CostmapToDynamicObstacles.cfg
│   │   │       ├── CostmapToLinesDBSMCCH.cfg
│   │   │       ├── CostmapToLinesDBSRANSAC.cfg
│   │   │       ├── CostmapToPolygonsDBSConcaveHull.cfg
│   │   │       └── CostmapToPolygonsDBSMCCH.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── costmap_converter
│   │   │       ├── costmap_converter_interface.h
│   │   │       ├── costmap_to_dynamic_obstacles
│   │   │       │   ├── background_subtractor.h
│   │   │       │   ├── blob_detector.h
│   │   │       │   ├── costmap_to_dynamic_obstacles.h
│   │   │       │   └── multitarget_tracker
│   │   │       │       ├── Ctracker.h
│   │   │       │       ├── defines.h
│   │   │       │       ├── HungarianAlg.h
│   │   │       │       ├── Kalman.h
│   │   │       │       └── README.md
│   │   │       ├── costmap_to_lines_convex_hull.h
│   │   │       ├── costmap_to_lines_ransac.h
│   │   │       ├── costmap_to_polygons_concave.h
│   │   │       ├── costmap_to_polygons.h
│   │   │       └── misc.h
│   │   ├── msg
│   │   │   ├── ObstacleArrayMsg.msg
│   │   │   └── ObstacleMsg.msg
│   │   ├── package.xml
│   │   ├── plugins.xml
│   │   ├── README.md
│   │   ├── src
│   │   │   ├── costmap_converter_node.cpp
│   │   │   ├── costmap_to_dynamic_obstacles
│   │   │   │   ├── background_subtractor.cpp
│   │   │   │   ├── blob_detector.cpp
│   │   │   │   ├── costmap_to_dynamic_obstacles.cpp
│   │   │   │   └── multitarget_tracker
│   │   │   │       ├── Ctracker.cpp
│   │   │   │       ├── HungarianAlg.cpp
│   │   │   │       ├── Kalman.cpp
│   │   │   │       └── README.md
│   │   │   ├── costmap_to_lines_convex_hull.cpp
│   │   │   ├── costmap_to_lines_ransac.cpp
│   │   │   ├── costmap_to_polygons_concave.cpp
│   │   │   └── costmap_to_polygons.cpp
│   │   └── test
│   │       ├── costmap_polygons.cpp
│   │       └── costmap_polygons.test
│   ├── lidar_preprocess
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── lidar_preprocess.launch
│   │   ├── package.xml
│   │   └── src
│   │       └── lidar_preprocess.py
│   ├── move_base
│   │   ├── cfg
│   │   │   └── MoveBase.cfg
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── move_base
│   │   │       └── move_base.h
│   │   ├── launch
│   │   │   ├── base_local_planner_params.yaml
│   │   │   ├── costmap_common_params.yaml
│   │   │   ├── global_costmap_params.yaml
│   │   │   ├── local_costmap_params.yaml
│   │   │   └── move_base.launch
│   │   ├── package.xml
│   │   ├── planner_test.xml
│   │   └── src
│   │       ├── coverage_path_convert.py
│   │       ├── move_base.cpp
│   │       └── move_base_node.cpp
│   └── slam_gmapping
│       ├── gmapping
│       │   ├── CHANGELOG.rst
│       │   ├── CMakeLists.txt
│       │   ├── launch
│       │   │   └── slam_gmapping_pr2.launch
│       │   ├── nodelet_plugins.xml
│       │   ├── package.xml
│       │   ├── src
│       │   │   ├── main.cpp
│       │   │   ├── nodelet.cpp
│       │   │   ├── replay.cpp
│       │   │   ├── slam_gmapping.cpp
│       │   │   └── slam_gmapping.h
│       │   └── test
│       │       ├── basic_localization_laser_different_beamcount.test
│       │       ├── basic_localization_stage.launch
│       │       ├── basic_localization_stage_replay2.launch
│       │       ├── basic_localization_stage_replay.launch
│       │       ├── basic_localization_symmetry.launch
│       │       ├── basic_localization_upside_down.launch
│       │       ├── rtest.cpp
│       │       └── test_map.py
│       ├── README.md
│       └── slam_gmapping
│           ├── CHANGELOG.rst
│           ├── CMakeLists.txt
│           └── package.xml
└── tool
    ├── test_scripts
    │   ├── association_test.py
    │   ├── tf_freq_test.py
    │   └── tracker_test.py
    └── useful_commands.sh
```