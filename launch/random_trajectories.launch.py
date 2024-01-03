#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # Node for Drone 2
    model_name = {'gz_model_name': 'x500_d435'}
    autostart_id = {'px4_autostart_id': '4020'}
    instance_id = {'instance_id': '0'}
    xpos = {'xpos': '4.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.1'}
    headless= {'headless' : '0'}

    # Namespace
    ns='target'

    # PX4 SITL + Spawn x3
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_dataset_generation_ros'),
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': '0.0'
        }.items()
    )

    # MAVROS
    file_name = 'target_px4_pluginlists.yaml'
    package_share_directory = get_package_share_directory('uav_dataset_generation_ros')
    plugins_file_path = os.path.join(package_share_directory, file_name)
    file_name = 'target_px4_config.yaml'
    config_file_path = os.path.join(package_share_directory, file_name)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_dataset_generation_ros'),
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'mavros_namespace' :ns+'/mavros',
            'tgt_system': '1',
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'target/base_link',
            'odom_frame': 'target/odom',
            'map_frame': 'map'
        }.items()
    )    

    # Static TF map(or world) -> local_pose_ENU
    map_frame = 'map'
    odom_frame= 'odom'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), '0', '0', '0', '0', map_frame, ns+'/'+odom_frame],
    )

    # Transport rgb and depth images from GZ topics to ROS topics    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node_depthcam',
        executable='parameter_bridge',
        arguments=['/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                   '/d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

                   '--ros-args', '-r', '/d435/depth_image:='+ns+'/depth_image',
                   '-r', '/d435/image:='+ns+'/image',
                   '-r', '/d435/points:='+ns+'/points',
                   '-r', '/d435/camera_info:='+ns+'/camera_info'
                   ],
    )    
    random_trajectories_node = Node(
        package='uav_dataset_generation_ros',
        executable='execute_random_trajectories',
        output='screen',
        name='execute_random_trajectories',
        namespace=ns,
        parameters=[{'system_id': 1},
                    {'radius_bounds': [5.0, 15.0]},
                    {'omega_bounds': [1.5,3.0]},
                    {'xyz_bound_min': [-40.0, -40.0, 5.0]},
                    {'xyz_bound_max': [40.0, 40.0, 10.0]},
                    {'num_traj': 200},
                    {'traj_2D': True},
                    {'traj_directory': '/home/user/shared_volume/gazebo_trajectories/'},
                    {'file_name': 'gazebo_trajectory2D'},
        ],
        remappings=[
            ('mavros/state', 'mavros/state'),
            ('mavros/local_position/odom', 'mavros/local_position/odom'),
            ('mavros/setpoint_raw/local', 'mavros/setpoint_raw/local')
        ]
    )

    ld.add_action(gz_launch)
    ld.add_action(map2pose_tf_node)
    ld.add_action(mavros_launch)
    ld.add_action(random_trajectories_node)
    ld.add_action(ros_gz_bridge)
    return ld
