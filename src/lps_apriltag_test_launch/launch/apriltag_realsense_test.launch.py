from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_size = LaunchConfiguration('tag_size')
    tag_family = LaunchConfiguration('tag_family')

    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                remappings=[
                    ('image', image_topic),
                    ('camera_info', camera_info_topic),
                ],
                parameters=[{
                    # Isaac ROS AprilTag ROS params
                    'size': ParameterValue(tag_size, value_type=float),
                    'tag_family': tag_family,
                    'max_tags': 64,
                    'tile_size': 4,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/infra1/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/infra1/camera_info'),
        DeclareLaunchArgument('tag_size', default_value='0.12'),     # 8 cm tag => 0.08
        DeclareLaunchArgument('tag_family', default_value='tag36h11'),
        container,
    ])
