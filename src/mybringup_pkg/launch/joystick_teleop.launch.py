import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
        
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("mybringup_pkg"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("mybringup_pkg"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]                    
    )     
    
    twist_topics = os.path.join(get_package_share_directory("mybringup_pkg"),"config","twist_mux_topics.yaml")
    twist_locks = os.path.join(get_package_share_directory("mybringup_pkg"),"config","twist_mux_topics.yaml")
    twist_joy = os.path.join(get_package_share_directory("mybringup_pkg"),"config","twist_mux_topics.yaml")
    #print("Path:", twist_topics),
    
    twist_mux_launch = IncludeLaunchDescription(
            os.path.join(get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "mybots_controller/cmd_vel", 
            "config_locks": twist_locks,
            "config_topics": twist_topics,
            "config_joy": twist_joy,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="myutils",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
         #   twist_mux_launch,
          #  twist_relay_node,

        ]
    )
