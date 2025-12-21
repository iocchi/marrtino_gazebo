#!/bin/bash

SESSION=0

CMD="docker exec -it marrtino_gazebo "

$CMD tmux rename-window -t $SESSION:0 'robot'
$CMD tmux new-window -t $SESSION:1 -n 'http'
$CMD tmux new-window -t $SESSION:2 -n 'ws'
$CMD tmux new-window -t $SESSION:3 -n 'world'
$CMD tmux new-window -t $SESSION:4 -n 'gui'
$CMD tmux new-window -t $SESSION:5 -n 'lsb'

$CMD tmux send-keys -t $SESSION:0 'cd ~/src/marrtino_gazebo/bin' C-m
$CMD tmux send-keys -t $SESSION:0 './smarrtino.bash' C-m

sleep 3

$CMD tmux send-keys -t $SESSION:1 'cd /var/log/nginx' C-m
$CMD tmux send-keys -t $SESSION:1 'tail -f access.log'

sleep 3

$CMD tmux send-keys -t $SESSION:2 'cd ~/src/marrtino_gazebo/bin' C-m
$CMD tmux send-keys -t $SESSION:2 './start_codeserver.bash'

$CMD tmux send-keys -t $SESSION:3 'cd ~/src/marrtino_gazebo/marrtino_gazebo/src' C-m
$CMD tmux send-keys -t $SESSION:3 'python gz_objects.py -a objs.conf'

$CMD tmux send-keys -t $SESSION:4 'cd ~/src/marrtino_gazebo/bin' C-m
$CMD tmux send-keys -t $SESSION:4 '#./smarrtino_gui.bash' C-m

$CMD tmux send-keys -t $SESSION:5 'cd ~/src/marrtino_gazebo/bin/lsb' C-m
$CMD tmux send-keys -t $SESSION:5 'ls' C-m

sleep 3

# http://localhost:3080/code.html / test1.html

echo "Done"


