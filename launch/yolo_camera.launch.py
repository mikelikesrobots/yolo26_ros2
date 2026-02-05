from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        "model_name", default_value="yolo26n.pt", description="YOLO model filename"
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
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{"pixel_format": "mjpeg2rgb"}],
    )

    yolo_node = Node(
        package="yolo26_ros2",
        executable="yolo26_ros2",
        name="yolo_detector",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": LaunchConfiguration("detections_topic"),
                "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                "model_name": LaunchConfiguration("model_name"),
            }
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
