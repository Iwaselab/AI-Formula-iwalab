#!/bin/bash

# tmuxセッション名
SESSION=masamin_macro

# セッション開始
tmux new-session -d -s $SESSION

# 1つ目のウィンドウ: colcon build & ros2 bag play
tmux send-keys -t $SESSION "cd ~/workspace/masamin" C-m
tmux send-keys -t $SESSION "colcon build" C-m
tmux send-keys -t $SESSION "source install/setup.bash" C-m
tmux send-keys -t $SESSION "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION "ros2 bag play /home/fuga/AIFMovie/tdu_4_vehicle_info/ --loop" C-m

# 2つ目のウィンドウ: object_road_detector
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:1 "cd ~/workspace/masamin" C-m
tmux send-keys -t $SESSION:1 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:1 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:1 "export PYTHONPATH=/home/fuga/workspace/masamin/src/aiformula/perception/yolop:\$PYTHONPATH" C-m
tmux send-keys -t $SESSION:1 "export PYTHONPATH=/home/fuga/workspace/masamin/src/aiformula/perception/yolop/yolop:\$PYTHONPATH" C-m
tmux send-keys -t $SESSION:1 "ros2 launch object_road_detector object_road_detector.launch.py" C-m

# 3つ目のウィンドウ: vehicle_tf_broadcaster
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:2 "cd ~/workspace/masamin" C-m
tmux send-keys -t $SESSION:2 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:2 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:2 "ros2 launch sample_vehicle vehicle_tf_broadcaster.launch.py" C-m

# 4つ目のウィンドウ: lane_line_publisher
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:3 "cd ~/workspace/masamin" C-m
tmux send-keys -t $SESSION:3 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:3 "ros2 launch lane_line_publisher lane_line_publisher.launch.py debug:=true" C-m

# 5つ目のウィンドウ: rviz2
tmux new-window -t $SESSION
tmux send-keys -t $SESSION:4 "cd ~/workspace/masamin" C-m
tmux send-keys -t $SESSION:4 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION:4 "rviz2 -d ~/workspace/ros2_ws/src/aiformula/perception/lane_line_publisher/rviz/lane_line_publisher.rviz" C-m

# tmuxセッションにアタッチ
tmux attach-session -t $SESSION