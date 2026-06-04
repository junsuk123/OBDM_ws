from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="landing_perception",
                executable="perception_adapter_node",
                name="perception_adapter",
                output="screen",
            ),
            Node(
                package="landing_state_estimation",
                executable="state_estimator_node",
                name="landing_state_estimator",
                output="screen",
            ),
            Node(
                package="landing_ontology_reasoner",
                executable="ontology_reasoner_node",
                name="landing_ontology_reasoner",
                output="screen",
            ),
            Node(
                package="landing_path_generator",
                executable="path_generator_node",
                name="landing_path_generator",
                output="screen",
            ),
            Node(
                package="landing_safety_layer",
                executable="safety_layer_node",
                name="landing_safety_layer",
                output="screen",
            ),
            Node(
                package="landing_controller",
                executable="landing_controller_node",
                name="landing_controller",
                output="screen",
            ),
        ]
    )
