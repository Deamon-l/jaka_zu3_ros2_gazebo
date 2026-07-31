"""Detect the red Gazebo target and publish its 3D position in world."""

import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
# Registers PointStamped support with tf2_ros.Buffer.transform.
import tf2_geometry_msgs  # noqa: F401
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        self.declare_parameter('rgb_topic', '/wrist_camera/rgb_image')
        self.declare_parameter('depth_topic', '/wrist_camera/depth_image')
        self.declare_parameter(
            'camera_info_topic', '/wrist_camera/rgb_image/camera_info')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('marker_topic', '/detected_target_marker')
        self.declare_parameter('min_area', 40.0)
        self.declare_parameter('image_width', 128)
        self.declare_parameter('image_height', 96)
        self.declare_parameter('horizontal_fov', 1.047)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_depth = None
        self.camera_info = None
        self.last_status_log = self.get_clock().now()

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.info_callback, 10)
        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value,
            self.rgb_callback, 10)
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter('marker_topic').value, 10)
        self.point_pub = self.create_publisher(
            PointStamped, '/detected_target_point', 10)
        self.get_logger().info('Target detector started')

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def rgb_callback(self, rgb_msg):
        if self.latest_depth is None:
            self.log_status('waiting for depth image')
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(
                self.latest_depth, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return

        if depth.ndim != 2:
            self.get_logger().warning('Depth image is not single-channel')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Red wraps around the HSV hue boundary, so use two ranges.
        mask = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
        mask |= cv2.inRange(hsv, (170, 100, 80), (179, 255, 255))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.get_parameter('min_area').value:
            return
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        depth_value = self.sample_depth(depth, u, v)
        if not math.isfinite(depth_value) or depth_value <= 0.0:
            return

        fx, fy, cx, cy = self.intrinsics()
        point = PointStamped()
        point.header = rgb_msg.header
        # Gazebo scopes sensor headers under the spawned model name. Use the
        # URDF frame instead, since that is the frame published by RSP/TF.
        point.header.frame_id = self.get_parameter('camera_frame').value
        point.point.x = (u - cx) * depth_value / fx
        point.point.y = (v - cy) * depth_value / fy
        point.point.z = depth_value

        target_frame = self.get_parameter('target_frame').value
        try:
            world_point = self.tf_buffer.transform(
                point, target_frame, timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warning(f'TF to {target_frame} unavailable: {exc}')
            return
        self.point_pub.publish(world_point)
        self.publish_marker(world_point)
        self.get_logger().info(
            f'red target: pixel=({u},{v}) depth={depth_value:.3f}m '
            f'world=({world_point.point.x:.3f}, '
            f'{world_point.point.y:.3f}, {world_point.point.z:.3f})',
            throttle_duration_sec=1.0)

    def intrinsics(self):
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            info = self.camera_info
            return info.k[0], info.k[4], info.k[2], info.k[5]
        width = float(self.get_parameter('image_width').value)
        height = float(self.get_parameter('image_height').value)
        fov = float(self.get_parameter('horizontal_fov').value)
        focal = width / (2.0 * math.tan(fov / 2.0))
        self.log_status('CameraInfo unavailable; using SDF intrinsics fallback')
        return focal, focal, width / 2.0, height / 2.0

    def log_status(self, message):
        now = self.get_clock().now()
        if (now - self.last_status_log).nanoseconds > 2_000_000_000:
            self.get_logger().warning(message)
            self.last_status_log = now

    @staticmethod
    def sample_depth(depth, u, v):
        y0, y1 = max(0, v - 2), min(depth.shape[0], v + 3)
        x0, x1 = max(0, u - 2), min(depth.shape[1], u + 3)
        values = depth[y0:y1, x0:x1].astype('float32').reshape(-1)
        values = values[values > 0.0]
        if values.size == 0:
            return float('nan')
        # Gazebo's R_FLOAT32 depth is metres; support uint16 millimetres too.
        value = float(sorted(values.tolist())[len(values) // 2])
        return value / 1000.0 if depth.dtype == 'uint16' else value

    def publish_marker(self, point):
        marker = Marker()
        marker.header = point.header
        marker.header.frame_id = self.get_parameter('target_frame').value
        marker.ns = 'detected_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=1.0).to_msg()
        self.marker_pub.publish(marker)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
