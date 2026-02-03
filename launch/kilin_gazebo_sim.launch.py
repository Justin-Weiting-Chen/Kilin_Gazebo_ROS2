
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

def generate_launch_description():
       
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    #gz_args = LaunchConfiguration('gz_args', default='')
    world_path = PathJoinSubstitution(
        [
            FindPackageShare('kilin_description'),
            'worlds',
            'stair_world.sdf',
        ]
    )
    robot_desc = launch_ros.descriptions.ParameterValue(# URDF via xacro
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('kilin_description'),
                    'urdf', 'Kilin.xacro.urdf']
                ),
            ]
        ),
        value_type=str
    )    
    robot_description = {'robot_description': robot_desc}
    
    # 1. spawn Gazebo simulator
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 
                'launch', 
                'gz_sim.launch.py'
            ])
        ]),
        # -r: (recreate the simulation)
        launch_arguments={
            'gz_args': [world_path, ' -r -v 4']
        }.items()
    )

    # 2. generate robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'kilin_robot',
                   '-topic', 'robot_description',
                   '-allow_renaming', 'true',
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )
    
    # 3. ROS 2 controller manager
    controllers_yaml_path = PathJoinSubstitution(
        [
            FindPackageShare('kilin_description'),
            'config',
            'kilin_gazebo_controllers.yaml',
        ]
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ALL_active_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[#'ALL_hip_position_controller',
                   #'ALL_hip_velocity_controller',
                   'ALL_hip_effort_controller',
                   #'ALL_steering_position_controller',
                   'ALL_steering_effort_controller',
                   #'FL_wheel_position_controller',
                   #'FR_wheel_position_controller',
                   #'RL_wheel_position_controller',
                   #'RR_wheel_position_controller',
                   'FL_wheel_velocity_controller',
                   'FR_wheel_velocity_controller',
                   'RL_wheel_velocity_controller',
                   'RR_wheel_velocity_controller',
                #    'FL_wheel_effort_controller',
                #    'FR_wheel_effort_controller',
                #    'RL_wheel_effort_controller',
                #    'RR_wheel_effort_controller',
                   '--param-file',
                   controllers_yaml_path,
                   ],
    )
    ALL_inactive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ALL_hip_position_controller',
                   'ALL_hip_velocity_controller',
                   #'ALL_hip_effort_controller',
                   'ALL_steering_position_controller',
                   #'ALL_steering_effort_controller',
                   'FL_wheel_position_controller',
                   'FR_wheel_position_controller',
                   'RL_wheel_position_controller',
                   'RR_wheel_position_controller',
                #    'FL_wheel_velocity_controller',
                #    'FR_wheel_velocity_controller',
                #    'RL_wheel_velocity_controller',
                #    'RR_wheel_velocity_controller',
                   'FL_wheel_effort_controller',
                   'FR_wheel_effort_controller',
                   'RL_wheel_effort_controller',
                   'RR_wheel_effort_controller',
                   '--param-file',
                   controllers_yaml_path,
                   '--stopped'
                   ],
    )
    
    # 4. Kilin Coordinator Node
    coordinator_node = Node(
        package='kilin_description',
        executable='Kilin_Coordinator', # same in CMakeLists.txt
        name='Kilin_Coordinator',
        output='screen',
        parameters=[
            {'use_sim_time': True},  
            {'update_rate': 100}    
        ],
        remappings=[
            ('/joint_states', '/joint_states') 
        ]
    )
    
    # 5. Robot State Publisher
    # Load content of URDF into /robot_description 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 6. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Hip_init_cmd = ExecuteProcess(
    # cmd=['ros2', 'topic', 'pub', '-once', # 
    #      '/ALL_hip_effort_controller/commands', 
    #      'std_msgs/msg/Float64MultiArray', 
    #      '{data: [100.0, 100.0, 100.0, 100.0]}'],
    # output='screen'
    # )
    
    
    # Combine into LaunchDescription
    return LaunchDescription([

        gazebo_server,
        spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ALL_inactive_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ALL_inactive_controller_spawner,
                on_exit=[ALL_active_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ALL_active_controller_spawner,
                on_exit=[coordinator_node],
            )
        ),
        bridge,
        robot_state_publisher_node,
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
