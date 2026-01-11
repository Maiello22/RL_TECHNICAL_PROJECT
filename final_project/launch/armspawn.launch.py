from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def spawn_robot(name, namespace, x_pose, y_pose, z_pose, yaw_pose, xacro_file, yaml_file):
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
            '-z', str(z_pose),
            '-Y', str(yaw_pose)
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
        gripper_after_position,
    ]

def generate_launch_description():
    
    robot1_nodes = spawn_robot(
        name='arm',
        namespace='arm',
        x_pose=-9.0,
        y_pose=0.0,
        z_pose=0.10,
        yaw_pose=0.0,
        xacro_file='armando/arm.urdf.xacro',
        yaml_file='armando/armando_controllers.yaml'
    )

    # Static TF: map -> arm/odom
    static_tf_map_to_arm_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_arm_odom',
        arguments=[
            '0', '2', '0',
            '0', '0', '0', '1',
            'map',
            'arm/odom'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Bridges
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_cmd_vel_bridge',
        arguments=[
            '/arm/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_odom_bridge',
        arguments=[
            '/arm/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/arm/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            ('/arm/tf', '/tf'),
        ],
        output='screen'
    )
    
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_lidar_bridge',
        arguments=[
            '/arm/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )




    return LaunchDescription([
        *robot1_nodes,
        static_tf_map_to_arm_odom,
        bridge_cmd_vel,
        bridge_odom,
        bridge_lidar,

    ])