from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        "model_name", default_value="yolo26n-seg.pt", description="YOLO model filename"
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/image_raw", description="Image topic name"
    )

    detections_topic_arg = DeclareLaunchArgument(
        "detections_topic", default_value="/detections", description="Detections topic name"
    )

    annotated_image_topic_arg = DeclareLaunchArgument(
        "annotated_image_topic",
        default_value="/image_annotated",
        description="Annotated image topic name",
    )

    usb_cam_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
    )

    yolo_node = Node(
        package="yolo26_ros2",
        executable="yolo26_ros2",
        name="yolo_detector",
        parameters=[{
            "model_name": LaunchConfiguration("model_name"),
        }],
        remappings=[
            ("/yolo_detector/image_raw", "/camera/image_raw"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_name_arg,
            image_topic_arg,
            detections_topic_arg,
            annotated_image_topic_arg,
            usb_cam_node,
            yolo_node,
        ]
    )
