#!/bin/bash

# tmuxセッション名
SESSION=ros2_ws_git_macro

# セッション開始
tmux new-session -d -s $SESSION

# 0つ目のウィンドウ: gazebo_simulator
tmux send-keys -t $SESSION "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION "source install/setup.bash" C-m
tmux send-keys -t $SESSION "ros2 launch sample_simulator gazebo_simulator.launch.py" C-m 

# 1つ目のウィンドウ: rviz2
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:1 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:1 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:1 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "rviz2 -d ~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/lane_line_publisher/rviz/lane_line_publisher.rviz" C-m

# 1つ目のウィンドウ: object_road_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:2 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:2 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:2 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "export PYTHONPATH=~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2:\$PYTHONPATH" C-m
tmux send-keys -t $SESSION:2 "sleep 15 && ros2 launch object_road_detector object_road_detector.launch.py" C-m

# 2つ目のウィンドウ: object_road_detector(yolop使用)
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:3 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:3 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:3 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "export PYTHONPATH=~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolop:\$PYTHONPATH" C-m
tmux send-keys -t $SESSION:3 "export PYTHONPATH=~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolop/yolop:\$PYTHONPATH" C-m
tmux send-keys -t $SESSION:3 "sleep 15 && ros2 launch object_road_detector_old object_road_detector_old.launch.py" C-m

# 3つ目のウィンドウ: detection_fusion
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:4 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:4 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:4 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "sleep 15 && ros2 launch detection_fusion detection_fusion.launch.py" C-m

# 4つ目のウィンドウ: vehicle_tf_broadcaster
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:5 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:5 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:5 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "sleep 15 && ros2 launch sample_vehicle vehicle_tf_broadcaster.launch.py" C-m

# 5つ目のウィンドウ: lane_line_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:6 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:6 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:6 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:6 "sleep 15 && ros2 launch lane_line_publisher lane_line_publisher.launch.py debug:=true" C-m

# 6つ目のウィンドウ: yolo_object_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:7 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:7 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:7 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:7 "sleep 15 && ros2 launch yolo_object_detector yolo_object_detector.launch.py" C-m

# 7つ目のウィンドウ: gyro_odometry_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:8 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:8 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:8 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:8 "sleep 15 && ros2 launch odometry_publisher gyro_odometry_publisher.launch.py" C-m

# 8つ目のウィンドウ: object_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:9 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:9 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:9 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:9 "sleep 15 && ros2 launch object_publisher object_publisher.launch.py" C-m

# 10つ目のウィンドウ: extremum_seeking_mpc
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:10 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:10 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:10 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:10 "sleep 30 && ros2 launch extremum_seeking_mpc extremum_seeking_mpc.launch.py" C-m

# 11つ目のウィンドウ: twist_logger.py
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:11 "cd ~/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:11 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:11 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:11 "cd ~/workspace/ros2_ws_git/src/AI-Formula-iwalab" C-m
tmux send-keys -t $SESSION:11 "sleep 30 && python3 twist_logger.py --topic /aiformula_control/extremum_seeking_mpc/cmd_vel --output twist_log.csv" C-m

# # 11つ目のウィンドウ: road_surface_segmenter
# tmux new-window -t $SESSION
# tmux send-keys -t $SESSION:11 "cd ~/workspace/ros2_ws_git" C-m
# tmux send-keys -t $SESSION:11 "source /opt/ros/foxy/setup.bash" C-m
# tmux send-keys -t $SESSION:11 "source install/setup.bash" C-m
# tmux send-keys -t $SESSION:11 "export PYTHONPATH=~/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2:\$PYTHONPATH" C-m
# tmux send-keys -t $SESSION:11 "ros2 launch road_surface_segmenter road_surface_segmenter.launch.py" C-m

# tmuxセッションにアタッチ
tmux attach-session -t $SESSION