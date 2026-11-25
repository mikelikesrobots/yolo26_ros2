import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        image_topic = "~/image_raw"
        detections_topic = "~/detections"
        annotated_image_topic = "~/image_annotated"

        model_name = self.declare_parameter("model_name", "yolo26n-seg.pt").value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Load YOLO model from package share directory
        package_share_dir = get_package_share_directory("yolo26_ros2")
        full_model_path = os.path.join(package_share_dir, "models", model_name)

        self.get_logger().info(f"Loading YOLO model from: {full_model_path}")
        self.model = YOLO(full_model_path)
        self.get_logger().info("YOLO model loaded successfully")

        # Create subscriber and publishers
        self.img_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.det_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self.annotated_img_pub = self.create_publisher(Image, annotated_image_topic, 10)
        self.compressed_img_pub = self.create_publisher(
            CompressedImage, annotated_image_topic + "/compressed", 10
        )

        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info(f"Publishing detections to: {detections_topic}")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        img_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Run YOLO detection
        results = self.model(img_rgb, retina_masks=True, verbose=False)

        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Process each detection
        img_vis = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        for result in results:
            if result.masks is not None:
                overlay = img_vis.copy()
                for i, mask in enumerate(result.masks.data):
                    mask = mask.cpu().numpy().astype(bool)

                    # Generate instance-specific colour (BGR)
                    color = tuple(
                        int(c) for c in cv2.cvtColor(
                            np.uint8([[[i * 37 % 180, 255, 255]]]),
                            cv2.COLOR_HSV2BGR,
                        )[0, 0]
                    )

                    overlay[mask] = color

                # Draw segmentation masks with 50% alpha
                segm_weight = 0.5
                img_vis = cv2.addWeighted(overlay, segm_weight, img_vis, 1-segm_weight, 0)

            for box in result.boxes:
                detection = Detection2D()

                # Get bounding box coordinates (xyxy format)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Set bounding box center and size
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Draw bounding box onto image
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

                label = int(box.cls[0])
                label_readable = self.model.names[label]
                score = float(box.conf[0])

                # Set detection hypothesis (class and confidence)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(label)
                hypothesis.hypothesis.score = score

                text = f"{label_readable}: {score:.2f}"
                cv2.putText(
                    img_vis,
                    text,
                    (x1, max(y1 - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

        # Publish detections and annotated images
        self.det_pub.publish(detection_array)

        annotated_img_msg = self.bridge.cv2_to_imgmsg(cvim = img_vis, header = msg.header, encoding="bgr8")
        self.annotated_img_pub.publish(annotated_img_msg)

        compressed_image_msg = CompressedImage()
        compressed_image_msg.header = msg.header
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode(".jpg", img_vis)[1]).tobytes()
        self.compressed_img_pub.publish(compressed_image_msg)

        self.get_logger().debug(f"Published {len(detection_array.detections)} detections")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
