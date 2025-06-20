# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')

    pkg_share = FindPackageShare('marrtino_gazebo')


    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=TextSubstitution(text='marrtino'),
        description='Name of the robot to spawn.'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=TextSubstitution(text='empty.world'),
        description='Gazebo world to launch.'
    )
    
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')   

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [ pkg_share, 'urdf', 
                    PythonExpression( ["'", robot_name, ".urdf.xacro'" ]) ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [ pkg_share, 'config', 
            PythonExpression(["'", robot_name, "_controllers.yaml'"]) ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[ robot_description ]
    )

    node_marrtino_parameters = Node(
        package='marrtino_control',
        executable='marrtino_parameters',
        output='screen',
        emulate_tty=True,   # Crucial for some Python nodes to ensure proper I/O and executable lookup
        parameters=[
            {'robot_name': PythonExpression( [ "'", robot_name, "'" ])}, 
            {'test': 'test1'}, ]
    )


    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', robot_name, '-allow_renaming', 'true'],
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    marrtino_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            PythonExpression(["'", robot_name, "_controller'"]),
            '--param-file',
            robot_controllers,
            ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    '''
    SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', [ EnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
        PathJoinSubstitution([pkg_share, 'models']) ]
    ),
    SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', [ EnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
        PathJoinSubstitution([pkg_share, 'worlds']) ]
    ),
    '''

    return LaunchDescription([
        robot_name_arg,
        world_file_arg,
    
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),

            launch_arguments = [ 
                ('gz_args', [gz_args, ' -r -v 1 ', 
                    PathJoinSubstitution([pkg_share, 'worlds/', world_file]) ]),
                ('on_exit_shutdown', 'True' ),
             ] ),


        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[marrtino_controller_spawner],
            )
        ),
        bridge,
        node_marrtino_parameters,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])

