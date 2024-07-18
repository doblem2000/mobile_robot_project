ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/motion_control" -p parameter_name:="safety_override" -p parameter_value:="full"
ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/oakd" -p parameter_name:="rgb.i_keep_preview_aspect_ratio" -p parameter_value:=false
ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/oakd" -p parameter_name:="rgb.i_resolution" -p parameter_value:="4K"
ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/oakd" -p parameter_name:="rgb.i_width" -p parameter_value:=2560
ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/oakd" -p parameter_name:="rgb.i_height" -p parameter_value:=1440
ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/oakd" -p parameter_name:="rgb.i_fps" -p parameter_value:=30.0
ros2 run autonomous_navigation_challenge_2024 change_parameters --ros-args -p node_name:="/oakd" -p parameter_name:="rgb.i_preview_size" -p parameter_value:=1000