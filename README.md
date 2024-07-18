# Mobile Robots For Critical Missions - Project Work Group 9

This repository contains the code for the M.Sc course in Mobile Robots for Critical Missions of group 9, composed of:
- Langella Antonio: a.langella31@studenti.unisa.it
- Marsico Michele: m.marsico10@studenti.unisa.it
- Paolino Salvatore: s.paolino6@studenti.unisa.it
- Trotta Prisco: p.trotta12@studenti.unisa.it

# Steps to Run the Code on the Turtlebot
### Pre-requirements
- Ensure that both this repository and the [turtlebot4](https://github.com/AntoSave/turtlebot4) repository are cloned in the same workspace;
- Be connected to a Turtlebot4 robot with a working camera.


### Build the code
Clone the repository in a ROS2 workspace's `src` directory then compile with the following command
```bash
colcon build
```
Then, run each of the following commands in a separate terminal:
### Launch the Localization Node
```bash
source ./install/setup.bash
```
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=./src/turtlebot4/diem_turtlebot_ws/src/map/diem_map_clean.yaml 
```
Ensure that the blue message relative to the `bond_timer` is displayed in the terminal.

### Launch RViz
```bash
source ./install/setup.bash
```
```bash
ros2 launch turtlebot4_viz view_robot.launch.py 
```
Then open the RViz config `mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/navigation.rviz` and set the initial pose of the robot via RViz.

### Launch Navigation Stack
```bash
source ./install/setup.bash
```
```bash
ros2 launch turtlebot4_navigation nav2.launch.py params_file:=src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/nav2.yaml
```
Ensure that the blue message relative to the `bond_timer` is displayed in the terminal.

### Launch the perception node
```bash
ros2 run autonomous_navigation_challenge_2024 perception --ros-args -p pipeline:=qreader_3_windows -p debug:=true
```
The parameters serve the following purposes:
- `pipeline`: selects the perception pipeline to be used;
- `debug`: set this parameter to `true` if you want visualize turtlebot camera and the results of detections (this adds latency).

### Launch navigation node
```bash
ros2 run autonomous_navigation_challenge_2024 navigation
```
The following parameters can set:
- `perception_topic`: The topic where the perception data is published
- `mock_kidnap`: set this parameter to `true` if you want to activate the mock of the kidnap feature
- `show_topological_map`: If true, the topological map will be shown in RViz

# Steps to Run the Code in Gazebo

### Launch the simulation
Launch GAZEBO, Localization Node and RVIZ with our launch file:
```bash
ros2 launch diem_gazebo turtlebot4_ignition_no_dock.launch.py rviz:=true nav2:=true localization:=true map:=./src/turtlebot4/diem_turtlebot_ws/src/map/diem_map.yaml world:=diem_map params_file:=src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/nav2.yaml
```

### Launch the mock SignPost Publisher
```bash
ros2 run autonomous_navigation_challenge_2024 mock_signposts_publisher
```
In order to publish a mock signpost you have to press the `w`, `a`, `s`, `d`, `i`, `e` buttons (follow the instructions).

### Launch Mock Kidnap
```bash
ros2 run autonomous_navigation_challenge_2024 mock_kidnap
```
In order to activate the kidnap you have to press the `T` button, and if you want to stoop the kidnap you have to press `F` button.

### Launch Navigation Node
```bash
ros2 run autonomous_navigation_challenge_2024 navigation --ros-args -p mock_kidnap:=true
```
Parameters:
- `mock_kidnap`: set this parameter to `true` if you want to activate the mock of the kidnap feature


# How to create a new Topological Map

### Launch the localization node 
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=./src/turtlebot4/diem_turtlebot_ws/src/map/diem_map.yaml 
```

### Launch RViz
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
Then open the RViz config `mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/config/map_creation.rviz`.

Open the map editor node with:

```bash
ros2 run autonomous_navigation_challenge_2024 poly_creator
```

This node has several tools that can be selected by clicking on the `Continue` button in the RVizVisualToolsGUI's toolbar: `CREATE_POLYGON`, `CREATE_EDGE`, `EDIT_CENTER`, `EDIT_CARDINAL_POINTS`, `DELETE_POLYGON`. Every tool must be used by selecting the `PublishPoint` tool in RViz and then clicking on the map.

1. With the `CREATE_POLYGON` tool, you start creating the polygon by clicking on individual points on the map. To close the polygon, click on `Next` and the polygon will be created.
2. To create an edge between two polygons, you need to select the `CREATE_EDGE` tool, then click on the first polygon, then on the second, and enter N, S, E, W in the shell, depending on where the second polygon is located relative to the first.
3. To interrupt the creation of a polygon or an edge, just click on `Break`.
4. To modify the center of a node, you need to select the `EDIT_CENTER` tool and then click on the point where you want to place the new center of the node.
5. To modify the distance of the cardinal points of a node, you need to select the `EDIT_CARDINAL_POINTS` tool, select the node, and then enter the cardinal point to be modified and the new distance in the shell.
6. To delete a node, select the `DELETE_POLYGON` tool and then select the node to delete. NOTE: DO NOT DELETE THE FIRST NODE CREATED OTHERWISE THE ENTIRE MAP WILL BE DELETED

At the end of the creation, to save, you need to click on "Stop" and the file "polygon.dat" will be created in the folder from which we launched the node.
