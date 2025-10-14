#!/bin/bash

# tmuxセッション名
SESSION=img_macro

# セッション開始
tmux new-session -d -s $SESSION

# 1つ目のウィンドウ: colcon build & ros2 bag play
tmux send-keys -t $SESSION "cd ~/workspace/masamin" C-m
tmux send-keys -t $SESSION "colcon build" C-m
tmux send-keys -t $SESSION "source install/setup.bash" C-m
tmux send-keys -t $SESSION "source /opt/ros/foxy/setup.bash" C-m
tmux send-keys -t $SESSION "python3 /home/fuga/workspace/masamin/src/aiformula/photo_expansion/reverse_images.py /home/fuga/AIFMovie/images/original" C-m
tmux send-keys -t $SESSION "python3 /home/fuga/workspace/masamin/src/aiformula/photo_expansion/reverse_images_mask.py /home/fuga/AIFMovie/images/mask" C-m
tmux send-keys -t $SESSION "python3 /home/fuga/workspace/masamin/src/aiformula/photo_expansion/blight_images.py" C-m
tmux send-keys -t $SESSION "python3 /home/fuga/workspace/masamin/src/aiformula/photo_expansion/blight_images_mask.py" C-m
tmux send-keys -t $SESSION "python3 /home/fuga/workspace/masamin/src/aiformula/photo_expansion/generate_black_images.py" C-m
tmux send-keys -t $SESSION "python3 /home/fuga/workspace/masamin/src/aiformula/photo_expansion/export_json_from_images.py" C-m

# tmuxセッションにアタッチ
tmux attach-session -t $SESSION