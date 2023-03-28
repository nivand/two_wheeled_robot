import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import sys
# from nav2_common.launch import RewrittenYaml
from pathlib import Path

# This file launches elastic pose graph for localisation.
# The map being used is saved from a previous mapping session as .posegraph and .data files.
# In general this shouldn't be used as this form of localisation depends on a pre-existing odom -> base_link transform, which we don't have.

def generate_launch_description(argv=sys.argv[1:]):
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace = LaunchConfiguration('namespace')
    remappings = [
                #     ('/tf', 'tf'),
                #   ('/tf_static', 'tf_static')
                  ]

        
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_namespace = DeclareLaunchArgument(
            'namespace', default_value='lewis',
            description='Top-level namespace')

    push_ns = PushRosNamespace(namespace=namespace)

    start_box1_node = Node(
        parameters=[{'model_name': 'cyl1'},
                    {'v_x': -0.10},
                    {'v_y': 0.120},
                    {'min_x': 10.5},
                    {'min_y': 4.0},
                    {'max_x': 13.0},
                    {'max_y': 5.0},
                    ],
        package='two_wheeled_robot',
        executable='simple_dynamic_obstacle',
        name='box1_node',
        output='screen',
        remappings=remappings,
        )

    start_box2_node = Node(
        parameters=[{'model_name': 'cyl2'},
                    {'v_x': 0.25},
                    {'v_y': 0.0},
                    {'min_x': 2.50},
                    {'min_y': -9.50},
                    {'max_x': 10.50},
                    {'max_y': -9.50},
                    ],
        package='two_wheeled_robot',
        executable='simple_dynamic_obstacle',
        name='box2_node',
        output='screen',
        remappings=remappings,
        )
    
    start_box3_node = Node(
        parameters=[{'model_name': 'cyl3'},
                    {'v_x': 0.09},
                    {'v_y': 0.25},
                    {'min_x': 15.0},
                    {'min_y': -11.80},
                    {'max_x': 16.90},
                    {'max_y': -10.0},
                    ],
        package='two_wheeled_robot',
        executable='simple_dynamic_obstacle',
        name='box3_node',
        output='screen',
        remappings=remappings,
        )

    

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(push_ns)
    ld.add_action(declare_use_sim_time_argument)
    #ld.add_action(start_box1_node)
    ld.add_action(start_box2_node)
    ld.add_action(start_box3_node)
    #ld.add_action(start_box4_node)
    #ld.add_action(start_box5_node)

    return ld