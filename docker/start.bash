#!/bin/bash

SESSION=0

CMD="docker exec -it marrtino2_gz "

$CMD tmux rename-window -t $SESSION:0 'robot'
$CMD tmux new-window -t $SESSION:1 -n 'http'
$CMD tmux new-window -t $SESSION:2 -n 'ws'
$CMD tmux new-window -t $SESSION:3 -n 'world'
$CMD tmux new-window -t $SESSION:4 -n 'gui'

$CMD tmux send-keys -t $SESSION:0 'cd ~/src/marrtino_gazebo/bin' C-m
$CMD tmux send-keys -t $SESSION:0 './smarrtino.bash' C-m

sleep 3

# not needed wsserver starts http server on port 3080
#$CMD tmux send-keys -t $SESSION:1 'cd ~/src/marrtino_gazebo/bin' C-m
#$CMD tmux send-keys -t $SESSION:1 './start_http.bash' C-m

sleep 3

$CMD tmux send-keys -t $SESSION:2 'cd ~/src/marrtino_gazebo/bin' C-m
$CMD tmux send-keys -t $SESSION:2 './start_codeserver.bash'

sleep 3

$CMD tmux send-keys -t $SESSION:3 'cd ~/src/marrtino_gazebo/marrtino_gazebo/src' C-m
$CMD tmux send-keys -t $SESSION:3 'python gz_objects.py -a objs.conf'

sleep 3

$CMD tmux send-keys -t $SESSION:4 'cd ~/src/marrtino_gazebo/bin' C-m
$CMD tmux send-keys -t $SESSION:4 '#./smarrtino_gui.bash' C-m

sleep 3

# http://localhost:3080/code.html / test1.html

echo "Done"


