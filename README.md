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
    ./run_vnc.bash

Use a browser on `http://localhost:3000` to see the simulation.


# Test

Inside the container

* Window 1

    cd ros2_ws
    
    colcon build

    ros2 launch marrtino_gazebo marrtino.launch.py

You should see the gazebo simulator environment running. 

In case of issues about accesssing the hos X server (e.g., message `Cannot open display :0`)
try disabling X access control in the host terminal

    xhost -

    
* Window 2 (use CTRL-b c to create a new window in tmux)

    cd src/marrtino_gazebo/src
    
    python3 control.py

Enjoy the robot moving on a square... 
and its trajectory error due to open-loop control !

PLots about velocities, positions and the executed trajectory are shown.
Close the plot windows to terminate the script.



