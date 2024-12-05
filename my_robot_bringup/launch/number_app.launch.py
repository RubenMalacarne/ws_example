from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    remap_number_topic = ("number", "my_number")
    
    ##now add node but you need to import 
    number_publisher_node = Node(
        package="py_base_pkg",
        executable="num_publisher_node",   
        name = "my_number_publisher",
        remappings = [
            remap_number_topic
        ],
        parameters= [
            {"number_to_publish":4},
            {"puclish_frequency":5.0}
        ]
        
    )
    
    number_counter_node = Node (
        package="cpp_base_pkg",
        executable="NumberCounterNode",
        name = "my_number_counter",
        remappings = [
            remap_number_topic,
            ("number_count", "my_number_count")
        ]
    )
    


    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    
    
    
    return ld