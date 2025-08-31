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
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')


    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=TextSubstitution(text='marrtino'),
        description='Name of the robot to spawn.'
    )

    battery_x_factor_arg = DeclareLaunchArgument(
        'battery_x_factor',
        default_value=TextSubstitution(text='1.0'),
        description='Multiplying factor for battery X position in the robot base (default 1.0, with values <1 more slippery).'
    )

    control_interface_args = DeclareLaunchArgument(
        'control_interface',
        default_value=TextSubstitution(text='effort'),
        description='control interface [effort|velocity|position]'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=TextSubstitution(text='empty.world'),
        description='Gazebo world to launch.'
    )
    
    robot_name = LaunchConfiguration('robot_name')
    battery_x_factor = LaunchConfiguration('battery_x_factor')
    control_interface = LaunchConfiguration('control_interface')
    world_file = LaunchConfiguration('world_file')   

    individual_arm_control = LaunchConfiguration('individual_arm_control', default=True)

    pkg_share = FindPackageShare('marrtino_gazebo')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [ pkg_share, 'urdf', 
                    PythonExpression( ["'", robot_name, ".urdf.xacro'" ]) ]
            ),
            ' ',
            PythonExpression( ["'battery_x_factor:=", battery_x_factor, "'" ]) 
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
            {'battery_x_factor': PythonExpression( [ battery_x_factor ])}, 
            {'control_interface': PythonExpression( [ "'", control_interface, "'" ])}, 
            {'individual_arm_control': PythonExpression( [  individual_arm_control ])}, 
            {'world': PythonExpression( [ "'", world_file, "'" ])}, 
            ]
    )


    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', robot_name, '-allow_renaming', 'true',
                   '-z', '0.01', '-Y', '0.00' ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
            ],
    )


    has_arms = PythonExpression(["'", robot_name, "' in ['smarrtino','marrtino_2_arms'] and not ", individual_arm_control ])

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            PythonExpression( ["'arm_", control_interface, "_controller'" ] ),
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_arms),
    )

    has_arms_individual_control = PythonExpression(["'", robot_name, "' in ['smarrtino','marrtino_2_arms'] and ",  individual_arm_control ])

    left_shoulder_pitch_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            PythonExpression( ["'left_shoulder_pitch_", control_interface, "_controller'" ] ),
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_arms_individual_control),
    )

    right_shoulder_pitch_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            PythonExpression( ["'right_shoulder_pitch_", control_interface, "_controller'" ] ),
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_arms_individual_control),
    )

    has_head = PythonExpression(["'", robot_name, "' == 'smarrtino'" ])

    head_pan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            PythonExpression( ["'head_pan_", control_interface, "_controller'" ] ),
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_head),
    )

    head_tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            PythonExpression( ["'head_tilt_", control_interface, "_controller'" ] ),
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_head),
    )


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean',
            '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity@gz.msgs.Entity@gz.msgs.Boolean',

            #'/model/marrtino/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            PythonExpression( ["'/model/", robot_name, "/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'"] ),
            
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
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
        battery_x_factor_arg,
        control_interface_args,
        world_file_arg,
    
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),

            launch_arguments = [ 
                ('gz_args', [gz_args, ' -r -v 4 ', 
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
                on_exit=[diff_drive_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_controller_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_controller_spawner,
                on_exit=[left_shoulder_pitch_controller_spawner, right_shoulder_pitch_controller_spawner
],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_controller_spawner,
                on_exit=[head_pan_controller_spawner, head_tilt_controller_spawner],
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

