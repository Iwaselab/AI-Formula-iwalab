#!/bin/bash

# tmuxセッション名
SESSION=ros2_ws_git_macro

# セッション開始
tmux new-session -d -s $SESSION

# 0つ目のウィンドウ: ros2 bag play
tmux send-keys -t $SESSION "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION "source install/setup.bash" C-m
tmux send-keys -t $SESSION "ros2 bag play --loop /home/fuga/AIFMovie/tdu_4_vehicle_info/ --loop --topics /aiformula_sensing/zed_node/left_image/undistorted /aiformula_sensing/vehicle_info /aiformula_sensing/zed_node/imu" C-m 

# 1つ目のウィンドウ: object_road_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:1 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:1 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:1 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "export PYTHONPATH=~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2:\$PYTHONPATH" C-m
tmux send-keys -t $SESSION:1 "ros2 launch object_road_detector object_road_detector.launch.py" C-m

# 2つ目のウィンドウ: vehicle_tf_broadcaster
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:2 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:2 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:2 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "ros2 launch sample_vehicle vehicle_tf_broadcaster.launch.py" C-m

# 3つ目のウィンドウ: lane_line_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:3 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:3 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:3 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "ros2 launch lane_line_publisher lane_line_publisher.launch.py debug:=true" C-m

# 4つ目のウィンドウ: yolo_object_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:4 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:4 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:4 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "ros2 launch yolo_object_detector yolo_object_detector.launch.py" C-m

# 5つ目のウィンドウ: gyro_odometry_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:5 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:5 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:5 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "ros2 launch odometry_publisher gyro_odometry_publisher.launch.py" C-m

# 6つ目のウィンドウ: object_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:6 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:6 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:6 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:6 "ros2 launch object_publisher object_publisher.launch.py" C-m

# 7つ目のウィンドウ: rviz2
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:7 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:7 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:7 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:7 "rviz2 -d ~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/lane_line_publisher/rviz/lane_line_publisher.rviz" C-m

# # 8つ目のウィンドウ: road_surface_segmenter
# tmux new-window -t $SESSION
# tmux send-keys -t $SESSION:8 "cd ~/workspace/ros2_ws_git" C-m
# tmux send-keys -t $SESSION:8 "source /opt/ros/foxy/setup.bash" C-m
# tmux send-keys -t $SESSION:8 "source install/setup.bash" C-m
# tmux send-keys -t $SESSION:8 "export PYTHONPATH=~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2:\$PYTHONPATH" C-m
# tmux send-keys -t $SESSION:8 "ros2 launch road_surface_segmenter road_surface_segmenter.launch.py" C-m

# tmuxセッションにアタッチ
tmux attach-session -t $SESSION