import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_husky_description = launch_ros.substitutions.FindPackageShare(package='husky_description').find('husky_description')
    pkg_husky_gazebo = launch_ros.substitutions.FindPackageShare(package='husky_gazebo').find('husky_gazebo')
    pkg_husky_control = launch_ros.substitutions.FindPackageShare(package='husky_control').find('husky_control')
    pkg_husky_viz = launch_ros.substitutions.FindPackageShare(package='husky_viz').find('husky_viz')

    default_model_path = os.path.join(pkg_husky_description, 'urdf/husky.urdf.xacro')
    world_path=os.path.join(pkg_husky_gazebo, 'worlds/empty_cones.world')
    default_rviz_config_path = os.path.join(pkg_husky_viz, 'rviz/custom_config.rviz')
    #default_rviz_config_path = os.path.join(pkg_share_tutorial, 'rviz/nav2_default_view.rviz')
    
    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"])
        
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description,{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
   
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen')
   
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    
    spawn_entity = launch_ros.actions.Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'husky', '-topic', 'robot_description'],
      output='screen',
      parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_husky_control, 'config/localization.yaml'),{'use_sim_time': True}]
    )


    robot_navsat_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='navsat_transform_node',
       name='navsat_transform_node',
       respawn='true',
       remappings=[('imu','imu/data'),('gps/fix', 'gps/data')],
       parameters=[{'magnetic_declination_radians': 0.0},{'yaw_offset': 1.5708},{'broadcast_utm_transform': True},{'publish_filtered_gps': True}]
    )
    
    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))
        
    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_scan_from_velodye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("pointcloud_to_laserscan"), 'launch', 'sample_pointcloud_to_laserscan_launch.py'])))  

    # Launch slam_toolbox to generate map
    launch_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), 'launch', 'online_async_launch.py'])))      

    # Launch nav2_bringup to move robot/ path finding
    launch_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), 'launch', 'navigation_launch.py'])))     
                             
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_joint_state_broadcaster,
        diffdrive_controller_spawn_callback,
        gzserver,
        gzclient,
        spawn_entity,
        robot_localization_node,
        rviz_node,
        launch_husky_control,
        launch_husky_teleop_base,
        launch_scan_from_velodye,
        #launch_slam_toolbox,
        #launch_nav2_bringup
    ])

