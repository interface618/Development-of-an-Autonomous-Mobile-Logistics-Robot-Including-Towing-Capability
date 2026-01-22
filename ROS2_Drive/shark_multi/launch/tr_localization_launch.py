import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    shark1_config = os.path.join(get_package_share_directory('shark_multi'), 'config', 'Traction_mode.yaml')

    ekf_yaml = os.path.join(get_package_share_directory('shark_multi'), 'config', 'ekf_shark1.yaml')

    map_file = os.path.join(get_package_share_directory('shark_multi'), 'map', 'center.yaml')

    
    

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}],
            remappings = [('map', '/map'),
                  ('map_updates', '/map_updates')]
        ),

        Node(
            namespace='shark1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[shark1_config],
            remappings=[('scan','/shark1/scan_front')]
        ),

        # Node(
        #     package='shark_multi',
        #     executable='initialpose',
        #     name='initialpose',
            
        # ),

        Node(
        namespace='shark1',
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],

        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'shark1/amcl']}]
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(get_package_share_directory('shark_multi'), 'rviz', 'nav2.rviz')],
        #     remappings = [('/tf', 'tf'),
        #           ('/tf_static', 'tf_static')]
        # )
    ])
