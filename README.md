# marrtino_gazebo

Ubuntu Noble Numbat (LTS 24.04)
https://releases.ubuntu.com/noble/

ROS2 Jazzy (LTS 2024-2029)
https://docs.ros.org/en/jazzy/

Gazebo Harmonic (LTS 2023-2028) 
https://gazebosim.org/docs/harmonic/



# Install

* docker engine

https://docs.docker.com/engine/

Install docker engine (not docker Desktop!!!)  (tested on v. 19.03, 20.10) 

Usually, this should work on Ubuntu distributions
    
        sudo apt install docker.io

or install from binaries

        https://docs.docker.com/engine/install/binaries/

See also 
[Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
In particular, add your user to the `docker` group and log out and in again, before proceeding.

        sudo usermod -aG docker $USER
        
Install docker compose plugin  (tested on v2.37)

https://docs.docker.com/compose/install/linux/

    DOCKER_CONFIG=${DOCKER_CONFIG:-$HOME/.docker}
    mkdir -p $DOCKER_CONFIG/cli-plugins
    curl -SL https://github.com/docker/compose/releases/download/v2.37.0/docker-compose-linux-x86_64 -o $DOCKER_CONFIG/cli-plugins/docker-compose
    chmod +x $DOCKER_CONFIG/cli-plugins/docker-compose


* For Nvidia drivers, install nvidia-docker2

[Nvidia docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
        

# Build

## Option 1: build images

    cd docker
    ./build.bash
    
In case of problems with building the image (possibly due to updates in the apt keys),
use the script

    ./build_no-cache.bash

# Option 2: pull images from dockerhub

    cd docker
    ./pull.bash


# Run

## Option 1: local graphic card

This option autodetects if nvidia drivers are present and uses nvidia runtime in docker

    cd docker
    ./run.bash

## Option 2: vnc 

    cd docker
    ./run.bash vnc

Use a browser on `http://localhost:3000` to see the simulation.

## Option 3: force X11

To force use of X11 driver and runc runtime

    cd docker
    ./run.bash x11



# Test

Inside the container

* Window 1

    cd ros2_ws
    
    colcon build

    ros2 launch marrtino_gazebo marrtino.launch.py

You should see the gazebo simulator environment running. 

In case of issues about accesssing the hos X server (e.g., message `Cannot open display :0`), try disabling X access control in the host terminal

    xhost +

then run the container again and repeat the above procedure.
 
Note: if you edit anything in `marrtino_gazebo`, relaunch with

    colcon build && ros2 launch marrtino_gazebo marrtino.launch.py

    
* Window 2 (use CTRL-b c to create a new window in tmux)

    cd ros2_ws
    
    colcon build

    ros2 run marrtino_control control


Enjoy the robot moving on a square... 
and its trajectory error due to open-loop control !

PLots about velocities, positions and the executed trajectory are shown.
Close the plot windows to terminate the script.

Note: if you edit anything in `marrtino_control`, relaunch with

    colcon build && ros2 run marrtino_control control



# Use

## Run the simulation

    ros2 launch marrtino_gazebo marrtino.launch.py robot_name:=<ROBOT_NAME> control_interface:=<CONTROL_INTERFACE> individual_arm_control:=<True|False> world_file:=<WORLD_FILE> battery_x_factor:=<battery_x_factor>


    ROBOT_NAME options:
    marrtino
    marrtino_2_arms
    smarrtino
    marrtina   TODO

    CONTROL_INTERFACE options:
    effort
    velocity
    position

    individual_arm_control:
    True: (default) one controller for each arm
    False: one controller for both arms

    WORLD_FILE options:
    empty

    battery_x_factor:  (only for marrtino robot type)
    1.0 (default) good position of the battery in the robot -> no slippery
    -1.0 bad position -> large slippery


Note: check simulated time is properly published, otherwise run gazebo again

    ros2 topic hz /clock


## Run the control program

Note: robot_name and control_interface are collected from `marrtino_parameters`


    ros2 run marrtino_control control --ros-args -p fn:=<FUNCTION> [ --ros-args -p plot:=<PLOT_STRING> ]

    `FUNCTION` options:
    square
    circle
    arms
    head
    all

    Optional `PLOT_STRING` values (multiple values can be used)
    velctrl
    odom
    

## Async behaviors

Note: so far available only for `control_interface:=position`

`setSpeed`, `setHeadPanPosition`,  `setHeadTiltPosition`,  `setLeftArmPosition`, `setRightArmPosition` (defined in `control.py`) can be run in asynchronous mode (in a separate thread) setting `_async=True`. In this mode, the function starts the behavior and immediately returns two values `thread` (a `Thread` object in which the behavior is running), `stop_event` (an `Event` object that can be used to stop the behavior externally).

Examples of use:

1) behaviors X and Y ends after time T

        th1,_ = setX(...., time = T, _async=True) 
        th2,_ = setY(...., time = T, _async=True) 
        # do something else ...
        th1.join()  # wait for thread th1 to complete
        th2.join()  # wait for thread th2 to complete


2) behaviors X ends at time T, behavior Y ends immediately after X 

        th1,_ = setX(...., time = T, _async=True) 
        th2,stev2 = setY(...., time = 10000, _async=True) 
        # do something else ...
        th1.join()  # wait for thread th1 to complete
        stev2.set() # send stop event for thread 2
        th2.join()  # wait for thread th2 to complete

3) behaviors X and Y end when a condition is met

        th1,stev1 = setX(...., time = 10000, _async=True) 
        th2,stev2 = setY(...., time = 10000, _async=True) 
        while (...):
            if condition:
                stev1.set()  # send stop event for thread 1
                stev2.set()  # send stop event for thread 2
        th1.join()  # wait for thread th1 to complete
        th2.join()  # wait for thread th2 to complete


During async execution, you can test if the thread is alive (i.e., the behavior is running) with `th.is_alive()`

        th1,_ = setX(...., time = T, _async=True)
        while (th1.is_alive()):
            # do something ...
        th1.join()  # wait for thread th1 to complete


# Rviz

    cd src/marrtino_gazebo/marrtino_gazebo/config
    ros2 run rviz2 rviz2 -d marrtino.rviz



# gz sim GUI


    export GZ_GUI_RESOURCE_PATH=${GZ_GUI_RESOURCE_PATH}:/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/

    cd ros2_ws

    ros2 launch marrtino_gazebo marrtino.launch.py robot_name:=smarrtino world_file:=empty.world gz_args:="--gui-config src/marrtino_gazebo/config/gui2.config"

## show camera image

    cd src/marrtino_gazebo/marrtino_gazebo/config
    gz gui -c gui-image.config





# Spawn objects in world

    cd src/marrtino_gazebo/marrtino_gazebo/src

    python gz_objects.py -h

Example

    python gz_objects.py -a objs.conf

Models: 
- https://app.gazebosim.org/fuel/models
- https://github.com/osrf/gazebo_models

# Create a new control script

## Create your new control script 

    cd src/marrtino_gazebo/marrtino_control/marrtino_control

    cp control.py control_<my_name>.py

## Edit setup.py

    cd src/marrtino_gazebo/marrtino_control/

Edit `setup.py` to add a line in the entry points like this

        'control_<my_name> = marrtino_control.control_<my_name>:main', 

## Edit your control script

    Edit `control_<my_name>.py` to implement a new controller function

    def my_new_controller():
        ...


## Run your new controller

    cd ros2_ws
    colcon build && ros2 run marrtino_control control_<my_name> --ros-args -p fn:=my_new_controller


# Run programs from web with websockets

Launch smarrtino robot, http server and code server

    cd src/marrtino_gazebo/bin
    ./smarrtino.bash

    cd src/marrtino_gazebo/bin
    ./start_server.bash

HTTP server running on port 3080, websocket on port 9876



Run gazebo GUI

    cd src/marrtino_gazebo/bin
    ./smarrtino_gui.bash


Connect with a browser at `http://localhost:8000/code.html`
and write programs in the code area.

Example:

        robot.forward(1)


# Basic Python high-level commands

        robot.forward(m)    move forward m [m]
        robot.backward(m)   move backward m [m]
        robot.left(r)       turn left r [deg]
        robot.right(r)      turn right r [deg]

        robot.setSpeed(lx,az,time,stopend=False)
                      lx: linear velocity [m/s]
				      az: angular velocity [rad/s]
                      time: time [s]
                      stopend: stop after motion 

        robot.pan(deg): positive left
        robot.tilt(deg): positive up

        robot.left_arm(deg): positive ahead
        robot.right_arm(deg): positive ahead

These commands can be sent asynchrnously, by adding the parameter `_async=True`

Example:  Raise both arms in parallel

        robot.left_arm(90, _async=True) # non-blocking
        robot.right_arm(90)             # blocking

Example:  Move arms in walking style

        def walking_arms(n):
            for i in range(2*n):
                a = 1 if i%2==0 else -1
                robot.left_arm(a*45, _async=True)
                robot.right_arm(-a*45)
                robot.sleep(1)

            robot.left_arm(0, _async=True)
            robot.right_arm(0)
            robot.sleep(1)


# websocket connection

Launch ROS2 bridge

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml  port:=9890


Open html/JS page connecting to ROS bridge

    www/marrtina02.html

or

    www/code.html


Test sending emotions to web face

    ros2 topic pub --once /social/emotion std_msgs/msg/String "data: 'normal'"


# xserver usage

How to show the simulator in a browser window

## Run in vnc mode

cd docker
./run_vnc

## Run gazebo simulator

Run gazebo server (headless, no GUI)

    cd src/marrtino_gazebo/bin
    ./smarrtino.bash

Run gazebo GUI

    cd src/marrtino_gazebo/bin
    ./smarrtino_gui.bash

## enter the xserver container

docker exec -it xserver bash


## Run x11vnc

Check that window is active

xwininfo -name "Gazebo Sim"

Then run the x11vnc and websockify servers

cd /app
./x11vnc_gzsim.bash


## Connect with browser

http://localhost:3001/vnc.html?autoconnect=1&resize=scale&quality=9&compression=2

NOTE: if window does not exist, check the desktop

http://localhost:3080/vnc.html?autoconnect=1&resize=scale


## Remote connection with ssh

From remote host use

    ssh -p 2222 robot@<host_running_the_container> -t tmux a
    password: robot


## gz sim control

check status

    gz topic --echo --topic /stats -n 1

pause

    gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'


start

    gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'




