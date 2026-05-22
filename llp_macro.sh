#!/bin/bash

# tmuxセッション名
SESSION=ros2_ws_git_macro

# セッション開始
tmux new-session -d -s $SESSION

# 1つ目のウィンドウ: colcon build & ros2 bag play
tmux send-keys -t $SESSION "cd /home/iwalab/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION "ros2 bag play --storage mcap --loop /home/iwalab/AIFMovie/test11_vehicle_info/test11_vehicle_info_0.mcap --topics /aiformula_sensing/zed_node/left_image/undistorted" C-m

# 2つ目のウィンドウ: object_road_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:1 "cd /home/iwalab/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:1 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:1 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "export PYTHONPATH=/home/iwalab/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2:\$PYTHONPATH" C-m #export PYTHONPATH=/home/iwalab/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/yolopv2:$PYTHONPATH
tmux send-keys -t $SESSION:1 "ros2 launch object_road_detector object_road_detector.launch.py" C-m

# 3つ目のウィンドウ: vehicle_tf_broadcaster
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:2 "cd /home/iwalab/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:2 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:2 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "ros2 launch sample_vehicle vehicle_tf_broadcaster.launch.py" C-m

# 4つ目のウィンドウ: lane_line_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:3 "cd /home/iwalab/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:3 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:3 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "ros2 launch lane_line_publisher lane_line_publisher.launch.py debug:=true" C-m

# 5つ目のウィンドウ: yolo_object_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:4 "cd /home/iwalab/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:4 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:4 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "ros2 launch yolo_object_detector yolo_object_detector.launch.py" C-m

# 5つ目のウィンドウ: rviz2
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:5 "cd /home/iwalab/workspace/ros2_ws_git" C-m
tmux send-keys -t $SESSION:5 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:5 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "rviz2 -d /home/iwalab/workspace/ros2_ws_git/src/AI-Formula-iwalab/perception/lane_line_publisher/rviz/lane_line_publisher.rviz" C-m

# tmuxセッションにアタッチ
tmux attach-session -t $SESSION