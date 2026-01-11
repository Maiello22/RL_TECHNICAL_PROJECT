from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def spawn_robot(name, namespace, x_pose, y_pose, z_pose, xacro_file, yaml_file):
    """Genera i nodi necessari per spawnare e controllare un robot in Gazebo"""
    
    pkg_share = FindPackageShare('final_project')
    yaml_path = PathJoinSubstitution([pkg_share, 'config', yaml_file])
    
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', xacro_file]),
        ' namespace:=', namespace,
        ' yaml_file:=', yaml_path
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-topic', f'/{namespace}/robot_description',
            '-x', str(x_pose),
            '-y', str(y_pose),
            '-z', str(z_pose)
        ],
        output='screen'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_controller',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )
    

    position_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[position_spawner]
        )
    )
    
    gripper_after_position = RegisterEventHandler(
        OnProcessExit(
            target_action=position_spawner,
            on_exit=[gripper_spawner]
        )
    )
    
  
    
    return [
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner,
        position_after_jsb,
        gripper_after_position
        
    ]


def generate_launch_description():
    
    pkg_share = FindPackageShare('final_project')
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'default.sdf'
    ])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r'],
        }.items()
    )
    
    # ✅ Robot 1: Spawn immediatamente
    robot1_nodes = spawn_robot(
        name='arm',
        namespace='arm',
        x_pose=0.0,
        y_pose=0.0,
        z_pose=0.0,
        xacro_file='armando/arm.urdf.xacro',
        yaml_file='armando/armando_controllers.yaml'
    )


    
    return LaunchDescription([
        gazebo,
        clock_bridge,
        *robot1_nodes,
         # ✅ Robot 2 con delay
    ])