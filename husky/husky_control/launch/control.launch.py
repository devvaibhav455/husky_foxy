from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    config_husky_ekf = PathJoinSubstitution(
        [FindPackageShare('husky_control'),
        'config',
        'localization.yaml'],
    )





    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_husky_ekf, {'use_sim_time':True}],
        remappings=[('imu', 'imu/data'),
                    ('gps/fix', 'gps/data'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')])
    ld.add_action(start_navsat_transform_cmd)



    #Disabled because it was publishing varying TF between odom and base_link even when robot was static due to which robot was not static in rviz visualization
    #Used for map to base_link TF
    node_ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        remappings=[('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose')],
        parameters=[config_husky_ekf, {'use_sim_time':True}],
        )
    # ld.add_action(node_ekf_global)


    #Used for odom to base_link TF
    node_ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        remappings=[('odometry/filtered', 'odometry/local'),
                ('/set_pose', '/initialpose')],
        parameters=[config_husky_ekf, {'use_sim_time':True}],
        )
    ld.add_action(node_ekf_local)

    pkg_husky_viz = FindPackageShare(package='husky_viz').find('husky_viz')
    default_rviz_config_path = os.path.join(pkg_husky_viz, 'rviz/custom_config_pcl.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time':True}])
    ld.add_action(rviz_node)

    primary_imu_enable = EnvironmentVariable('CPR_IMU', default_value='false')

    if (primary_imu_enable.perform(lc)) == 'true':
        config_imu_filter = PathJoinSubstitution(
            [FindPackageShare('husky_control'),
            'config',
            'imu_filter.yaml'],
        )
        node_imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[config_imu_filter]
        )
        ld.add_action(node_imu_filter)

    return ld
