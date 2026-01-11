import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.actions import SetEnvironmentVariable
import launch.conditions


def generate_launch_description():
    
    # ===== ARGOMENTI COMUNI =====
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Avvia Gazebo con GUI'
    )
    
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value='empty.sdf',  # Oppure usa leonardo_race_field.sdf
        description='World file da caricare'
    )
    
    # ===== GAZEBO (UNA SOLA ISTANZA) =====
    world_file = LaunchConfiguration('world_file')
    
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )
    
    # ===== MANIPOLATORE =====
    # Argomenti per il manipolatore
    arm_package_arg = DeclareLaunchArgument(
        'arm_urdf_package',
        default_value='armando_description',
        description='Package del manipolatore'
    )
    
    arm_model_arg = DeclareLaunchArgument(
        'arm_urdf_path',
        default_value='urdf/arm.urdf.xacro',
        description='Path URDF del manipolatore'
    )
    
    # Invece di includere description.launch.py, crea esplicitamente:
    arm_xacro = os.path.join(
        get_package_share_directory('armando_description'),
        'urdf',
        'arm.urdf.xacro'
    )

    arm_description = {
        "robot_description": ParameterValue(
            Command(['xacro ', arm_xacro]),
            value_type=str
        )
    }

    arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='arm',  # ← AGGIUNTO: namespace
        parameters=[arm_description, {"use_sim_time": True}]
    )
    
    # Spawna il manipolatore
    arm_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='arm_spawner',
        arguments=[
            '-param', '/arm/robot_description',
            '-name', 'manipulator_arm',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )
    
    # Controller per il manipolatore
    arm_controller_type_arg = DeclareLaunchArgument(
        'arm_controller_type',
        default_value='position',
        description='Tipo di controller: position o trajectory'
    )
    
    arm_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    arm_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression([
                "'", LaunchConfiguration('arm_controller_type'), "' == 'position'"
            ])
        )
    )
    
    arm_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression([
                "'", LaunchConfiguration('arm_controller_type'), "' == 'trajectory'"
            ])
        )
    )
    
    # Delay dei controller del manipolatore
    arm_delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_spawner,
            on_exit=[arm_joint_state_broadcaster, arm_position_controller, arm_trajectory_controller],
        )
    )
    
    # ===== DIFFERENTIAL DRIVE ROBOT =====
    # Path ai file del fra2mo
    fra2mo_xacro = os.path.join(
        get_package_share_directory('ros2_fra2mo'),
        "urdf",
        "fra2mo.urdf.xacro"
    )
    
    models_path = os.path.join(
        get_package_share_directory('ros2_fra2mo'),
        'models'
    )
    
    # Descrizione del fra2mo
    fra2mo_description = {
        "robot_description": ParameterValue(
            Command(['xacro ', fra2mo_xacro]),
            value_type=str
        )
    }
    
    # Robot state publisher per fra2mo (con namespace)
    fra2mo_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='fra2mo',
        parameters=[fra2mo_description, {"use_sim_time": True}]
    )
    
    # Joint state publisher per fra2mo
    fra2mo_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )
    
    # Spawna fra2mo
    fra2mo_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='fra2mo_spawner',
        arguments=[
            '-param', '/fra2mo/robot_description',
            '-name', 'fra2mo',
            '-allow_renaming', 'true',
            '-x', '2.0',  # Posiziona lontano dal manipolatore
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    # Bridge per fra2mo
    fra2mo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_bridge',
        namespace='fra2mo',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        remappings=[
            ('/cmd_vel', '/fra2mo/cmd_vel'),
        ],
        output='screen'
    )
    
    # Odom TF per fra2mo
    fra2mo_odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )
    
    # ===== LAUNCH DESCRIPTION =====
    return LaunchDescription([
        # Variabili d'ambiente
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        
        # Argomenti
        gui_arg,
        world_arg,
        arm_package_arg,
        arm_model_arg,
        arm_controller_type_arg,
        
        # Gazebo (unica istanza)
        gazebo_launch,
        
        # Manipolatore
        arm_state_publisher,
        arm_spawner,
        arm_delay_controllers,
        
        # Differential Drive Robot
        fra2mo_state_publisher,
        fra2mo_joint_publisher,
        fra2mo_spawner,
        fra2mo_bridge,
        fra2mo_odom_tf,
    ])