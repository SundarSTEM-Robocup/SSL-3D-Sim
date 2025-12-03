import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ssl_robot = get_package_share_directory('ssl_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Append to GZ_SIM_RESOURCE_PATH
    # We need to go one level up from pkg_ssl_robot to get to 'share'
    install_share_dir = os.path.join(pkg_ssl_robot, '..')
    
    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir,
        separator=':'
    )

    # World File
    world_file_name = 'ssl_field.world'
    world_path = os.path.join(pkg_ssl_robot, 'worlds', world_file_name)

    # Gazebo Sim (Harmonic)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 "{world_path}"'}.items(),
    )

    # URDF
    urdf_file_name = 'robot.urdf'
    urdf_path = os.path.join(pkg_ssl_robot, 'urdf', urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf_path]
    )

    # Spawn Entity (using ros_gz_sim create)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ssl_robot', '-topic', 'robot_description', '-x' , '0.5' , '-y' , '0.0' , '-z', '0.1'],
        output='screen'
    )

    # Bridge ROS <-> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # UI Controller Node
    gui_controller = Node(
        package='ssl_robot',
        executable='gui_controller.py',
        name='gui_controller',
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        gui_controller
    ])