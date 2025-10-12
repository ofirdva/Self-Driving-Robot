from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    #delcarations

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False" #using c++ as a default. to move to python change to true
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )
    use_simple_controller_arg = DeclareLaunchArgument(  #indicates which controller to choose
        "use_simple_controller",
        default_value="True"
    )
    
    use_python = LaunchConfiguration("use_python") #reading the value of the use_python arg and store it inside use_pythob var 
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
            ]
        )

    wheel_controller_spawner = Node(   
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bumperbot_controller", #using the diff driver controller
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions= [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
        ]
              
    ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[{"wheel_radius" : wheel_radius,
                            "wheel_separation" : wheel_separation}],
                condition = IfCondition(use_python)       
    ),

            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius" : wheel_radius,
                            "wheel_separation" : wheel_separation}],
                condition = UnlessCondition(use_python)       
            )
        ]
    )

    
    return LaunchDescription(
        [
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            use_simple_controller_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller
            
        ]
    )