import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

def generate_launch_description():

    # Percorsi ai file
    xacro_file_name = "fra2mo.urdf.xacro"
    xacro = os.path.join(get_package_share_directory('ros2_fra2mo'), "urdf", xacro_file_name)

    models_path = os.path.join(get_package_share_directory('ros2_fra2mo'), 'models')
    world_file = os.path.join(get_package_share_directory('ros2_fra2mo'), "worlds", "maiellomap.sdf")

    # Genera la descrizione del robot usando xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro]),value_type=str)}
    
    # use_sim_time_arg = DeclareLaunchArgument(
    #     'use_sim_time', default_value='true', description='Use simulation/Gazebo clock')

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": True}
            ]
    )
    
    # # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{"use_sim_time": True}]
    )

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value=[world_file, ' -r'],
                              description='path to world file'),)
    
    # Gazebo simulation launch description
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    position = [-9.0, -2.0, 0.100]

    # Define a Node to spawn the robot in the Gazebo simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),]
    )

    # ✅ Bridge per cmd_vel con remapping
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        ],
        output='screen'
    )

    # ✅ Bridge per odometry e tf con remapping
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_odom_bridge',
        arguments=[
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        ],
        output='screen'
    )

    # ✅ Bridge per sensori (lidar, clock) - questi restano invariati
    bridge_sensors = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_sensors_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )


    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/fra2mo/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/fra2mo/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace="fra2mo"
    )




    aruco_node = Node(
        package='aruco_ros',
        executable='marker_publisher', 
        name='aruco_marker_publisher',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.2,  
            'reference_frame': 'camera_optical_link',
            'camera_frame': 'camera_optical_link',
        
        }],
        remappings=[
            ('/camera_info', '/fra2mo/camera_info'),
            ('/image', '/fra2mo/camera'),
        ]
    )


    button_joint_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='button_joint_bridge',
        arguments=[
            '/aruco_cube_button/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )




 
    ign = [gazebo_ignition, gz_spawn_entity]
    nodes_to_start = [robot_state_publisher_node, 
                    joint_state_publisher_node, *ign, 
                    bridge_cmd_vel, bridge_odom, bridge_sensors, 
                    bridge_camera, odom_tf, ign_clock_bridge, aruco_node, button_joint_bridge]

    return LaunchDescription([SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))] + declared_arguments + nodes_to_start)