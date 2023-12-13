from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_node",
    )

    turtle_spawner_node = Node(
        package="my_turtle_sim_project",
        executable="turtle_spawner_node",
        name="turtle_spawner",

        parameters=[
            {"spawn_time": 1.0}
             ]
    )

    turtle_controller_node = Node(
        package="my_turtle_sim_project",
        executable = "turtle_controller",
        name = "turtle_controller",
        parameters=[
            {"catch_closest_turtle":True}
        ]
    )

        
    

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(turtle_spawner_node)
    
    return ld