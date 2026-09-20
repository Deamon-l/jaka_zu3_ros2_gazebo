"""Detect a red RGB-D target and publish its 3D position."""

import math
from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        self.declare_parameter('rgb_topic', '/wrist_camera/rgb_image')
        self.declare_parameter('depth_topic', '/wrist_camera/depth_image')
        self.declare_parameter(
            'camera_info_topic', '/wrist_camera/camera_info')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('use_latest_tf', True)
        self.declare_parameter('max_rgb_depth_delta', 0.25)
        self.declare_parameter('point_topic', '/detected_target_point')
        self.declare_parameter('marker_topic', '/detected_target_marker')
        self.declare_parameter('min_area', 40.0)
        self.declare_parameter('max_area', 6000.0)
        self.declare_parameter('min_saturation', 90)
        self.declare_parameter('min_value', 60)
        self.declare_parameter('min_depth', 0.10)
        self.declare_parameter('max_depth', 2.00)
        self.declare_parameter('min_depth_samples', 8)
        self.declare_parameter('image_width', 128)
        self.declare_parameter('image_height', 96)
        self.declare_parameter('horizontal_fov', 1.047)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_frames = deque(maxlen=5)
        self.camera_info = None
        self.last_status_log = self.get_clock().now()

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.info_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value,
            self.rgb_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter('marker_topic').value, 10)
        self.point_pub = self.create_publisher(
            PointStamped, self.get_parameter('point_topic').value, 10)
        self.get_logger().info('Target detector started')

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_frames.append(msg)

    def matching_depth(self, rgb_msg):
        """Return the depth frame nearest to RGB, rejecting stale pairs."""
        if not self.depth_frames:
            return None
        rgb_time = (rgb_msg.header.stamp.sec +
                    rgb_msg.header.stamp.nanosec * 1e-9)
        return min(
            self.depth_frames,
            key=lambda msg: abs(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                - rgb_time),
        )

    def rgb_callback(self, rgb_msg):
        depth_msg = self.matching_depth(rgb_msg)
        if depth_msg is None:
            self.log_status('waiting for depth image')
            return
        rgb_time = (rgb_msg.header.stamp.sec +
                    rgb_msg.header.stamp.nanosec * 1e-9)
        depth_time = (depth_msg.header.stamp.sec +
                      depth_msg.header.stamp.nanosec * 1e-9)
        if abs(rgb_time - depth_time) > float(
                self.get_parameter('max_rgb_depth_delta').value):
            self.log_status('waiting for synchronized RGB-D frames')
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return

        if depth.ndim != 2:
            self.get_logger().warning('Depth image is not single-channel')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        saturation = int(self.get_parameter('min_saturation').value)
        value = int(self.get_parameter('min_value').value)
        # Red wraps around the HSV hue boundary, so use two ranges.
        mask = cv2.inRange(hsv, (0, saturation, value), (10, 255, 255))
        mask |= cv2.inRange(hsv, (170, saturation, value), (179, 255, 255))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self.get_parameter('min_area').value)
        max_area = float(self.get_parameter('max_area').value)
        contours = [
            contour for contour in contours
            if min_area <= cv2.contourArea(contour) <= max_area
        ]
        if not contours:
            return
        contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        depth_value = self.sample_depth(
            depth, mask, contour,
            int(self.get_parameter('min_depth_samples').value))
        min_depth = float(self.get_parameter('min_depth').value)
        max_depth = float(self.get_parameter('max_depth').value)
        if (not math.isfinite(depth_value) or
                not min_depth <= depth_value <= max_depth):
            return

        fx, fy, cx, cy = self.intrinsics(bgr.shape[1], bgr.shape[0])
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
            # Gazebo publishes camera images just ahead of joint-state TF by
            # one simulation tick.  Use the newest camera pose in that case;
            # a static scene makes the resulting millimetre-scale difference
            # immaterial and prevents a persistent future-extrapolation error.
            transform_time = (
                Time() if self.get_parameter('use_latest_tf').value
                else Time.from_msg(point.header.stamp))
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                transform_time,
                timeout=Duration(seconds=0.1))
            world_point = do_transform_point(point, transform)
        except TransformException as exc:
            self.get_logger().warning(
                f'TF to {target_frame} unavailable: {exc}')
            return
        self.point_pub.publish(world_point)
        self.publish_marker(world_point)
        self.get_logger().info(
            f'red target: pixel=({u},{v}) depth={depth_value:.3f}m '
            f'world=({world_point.point.x:.3f}, '
            f'{world_point.point.y:.3f}, {world_point.point.z:.3f})',
            throttle_duration_sec=1.0)

    def intrinsics(self, image_width, image_height):
        """Return intrinsics scaled to the actual RGB image dimensions."""
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            info = self.camera_info
            source_width = float(info.width or image_width)
            source_height = float(info.height or image_height)
            scale_x = image_width / source_width
            scale_y = image_height / source_height
            return (
                info.k[0] * scale_x,
                info.k[4] * scale_y,
                info.k[2] * scale_x,
                info.k[5] * scale_y,
            )
        width = float(image_width or self.get_parameter('image_width').value)
        height = float(
            image_height or self.get_parameter('image_height').value)
        fov = float(self.get_parameter('horizontal_fov').value)
        focal = width / (2.0 * math.tan(fov / 2.0))
        self.log_status(
            'CameraInfo unavailable; using SDF intrinsics fallback')
        return focal, focal, width / 2.0, height / 2.0

    def log_status(self, message):
        now = self.get_clock().now()
        if (now - self.last_status_log).nanoseconds > 2_000_000_000:
            self.get_logger().warning(message)
            self.last_status_log = now

    @staticmethod
    def sample_depth(depth, mask, contour, min_samples=1):
        """Use the median of valid pixels inside the detected object."""
        object_mask = np.zeros_like(mask)
        cv2.drawContours(object_mask, [contour], -1, 255, thickness=-1)
        # Erode boundaries so background pixels do not bias the range reading.
        object_mask = cv2.erode(object_mask, None, iterations=1)
        if object_mask.shape != depth.shape:
            object_mask = cv2.resize(
                object_mask, (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST)
        values = depth[object_mask > 0].astype('float32').reshape(-1)
        values = values[values > 0.0]
        values = values[np.isfinite(values)] if values.size else values
        if values.size < min_samples:
            return float('nan')
        # Gazebo's R_FLOAT32 depth is metres; support uint16 millimetres too.
        value = float(np.median(values))
        return value / 1000.0 if np.issubdtype(depth.dtype, np.integer) else value

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
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
