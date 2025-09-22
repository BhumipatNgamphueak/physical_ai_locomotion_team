from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()
    number_of_leg = 6
    leg_ns = ["top_left_leg","middle_left_leg","bottom_left_leg","top_right_leg","middle_right_leg","bottom_right_leg"]
    #leg_ns = ["top_left_leg"]
    node_name = ["controller","forward_kinematic","gait_planning","inverse_kinematic","set_point","state_controller","trajectory_planning"]

    for namespace in leg_ns:
        for node in node_name:

            leg_node = Node(
                package = 'hexapod',
                namespace = namespace,
                executable = node + ".py",
                name = node
            )
            launch_description.add_action(leg_node)

    
    return launch_description
