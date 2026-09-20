"""Plan and execute a vision-gated top-down RGB-D pick in Gazebo."""

import math
from collections import deque
from statistics import median

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class TargetMotion(Node):
    """Observe first, then execute open, approach, descend, close, and lift."""

    POSE_1 = (0.0, 1.5707, -1.5707, 1.5707, 1.5707, 0.0)
    JOINT_NAMES = tuple(f'joint_{index}' for index in range(1, 7))

    def __init__(self):
        super().__init__('target_motion')
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('group_name', 'jaka_zu3')
        # The fingers are positioned from gripper_base_link.  gripper_tcp is
        # a separate tool marker and is intentionally not used for planning.
        self.declare_parameter('end_effector_link', 'gripper_base_link')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('target_topic', '/detected_target_point')
        self.declare_parameter('approach_offset_z', 0.14)
        # The detector reports the visible top surface.  At this offset the
        # base stays above the object while the fingers surround its centre.
        self.declare_parameter('grasp_offset_z', 0.0)
        self.declare_parameter('lift_offset_z', 0.16)
        # Finger joints move inward as their position increases.
        self.declare_parameter('open_width', 0.0)
        self.declare_parameter('closed_width', 0.010)
        self.declare_parameter('gripper_min_position', 0.0)
        self.declare_parameter('gripper_max_position', 0.025)
        self.declare_parameter('gripper_motion_time', 2.0)
        self.declare_parameter('grasp_settle_time', 0.5)
        self.declare_parameter('finger_sync_tolerance', 0.003)
        self.declare_parameter('finger_goal_tolerance', 0.005)
        self.declare_parameter('use_sim_attachment', False)
        self.declare_parameter('attachment_timeout', 3.0)
        self.declare_parameter('attach_topic', '/gripper/attach')
        self.declare_parameter('detach_topic', '/gripper/detach')
        self.declare_parameter(
            'attachment_state_topic', '/gripper/attached')
        self.declare_parameter('action_timeout', 90.0)
        self.declare_parameter('max_target_age', 0.75)
        self.declare_parameter('required_stable_detections', 5)
        self.declare_parameter('target_stability_tolerance', 0.015)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.10)
        self.declare_parameter(
            'planning_pipeline', 'pilz_industrial_motion_planner')
        self.declare_parameter('point_to_point_planner', 'PTP')
        self.declare_parameter('linear_planner', 'LIN')
        self.declare_parameter('velocity_scaling', 0.15)
        self.declare_parameter('acceleration_scaling', 0.08)
        self.declare_parameter('workspace_min_x', -0.75)
        self.declare_parameter('workspace_max_x', 0.75)
        self.declare_parameter('workspace_min_y', -0.75)
        self.declare_parameter('workspace_max_y', 0.75)
        self.declare_parameter('workspace_min_z', 0.0)
        self.declare_parameter('workspace_max_z', 0.75)
        self.declare_parameter('move_action', 'move_action')
        self.declare_parameter(
            'gripper_action',
            '/gripper_controller/follow_joint_trajectory')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_client = ActionClient(
            self, MoveGroup, self.get_parameter('move_action').value)
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('gripper_action').value)
        self.state = 'WAIT_SERVER'
        self.target = None
        sample_count = max(
            1, int(self.get_parameter('required_stable_detections').value))
        self.target_samples = deque(maxlen=sample_count)
        self.orientation = None
        self.settle_timer = None
        self.attachment_timer = None
        self.attachment_started_ns = None
        self.attachment_state_received = False
        self.object_attached = False
        self.startup_detach_attempts = 0
        self.startup_detach_timer = None
        self.finger_positions = {}
        self.create_subscription(
            PointStamped,
            self.get_parameter('target_topic').value, self.target_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.attach_publisher = None
        self.detach_publisher = None
        if self.get_parameter('use_sim_attachment').value:
            self.attach_publisher = self.create_publisher(
                Empty, self.get_parameter('attach_topic').value, 10)
            self.detach_publisher = self.create_publisher(
                Empty, self.get_parameter('detach_topic').value, 10)
            self.create_subscription(
                String,
                self.get_parameter('attachment_state_topic').value,
                self.attachment_state_callback,
                10)
            # DetachableJoint starts attached.  Send a short burst instead of
            # blocking all robot motion on its non-latched state topic.
            self.startup_detach_timer = self.create_timer(
                0.25, self.publish_startup_detach)
        if self.parameters_are_valid():
            self.create_timer(0.5, self.start)
        else:
            self.state = 'FAILED'
        execute = self.get_parameter('execute_motion').value
        mode = 'EXECUTE' if execute else 'PLAN_ONLY'
        self.get_logger().info(
            f'Waiting for MoveIt and gripper. mode={mode}; '
            'the robot will observe first, then act only after a stable '
            'target.')

    def start(self):
        if self.state != 'WAIT_SERVER':
            return
        if not self.move_client.server_is_ready():
            return
        if self.get_parameter('execute_motion').value and not (
                self.gripper_client.server_is_ready()):
            return
        if (self.get_parameter('use_sim_attachment').value and
                not self.attachment_state_received):
            self.get_logger().warning(
                'No startup attachment-state reply; continuing after '
                'sending detach commands.')
        self.state = 'MOVING_TO_OBSERVE'
        self.send_arm_goal(self.joint_goal(), self.observation_done)

    def observation_done(self, succeeded):
        if not succeeded:
            self.fail('Could not plan or execute pose_1')
            return
        if not self.get_parameter('execute_motion').value:
            self.state = 'DONE'
            self.get_logger().info(
                'Observation-pose plan succeeded. No motion was executed; '
                'set execute_motion:=true to run the pick sequence.')
            return
        self.state = 'WAIT_TARGET'
        self.get_logger().info('Observation pose reached. Waiting for target.')

    def target_callback(self, point):
        target_frame = self.get_parameter('target_frame').value
        if self.state != 'WAIT_TARGET':
            return
        if point.header.frame_id != target_frame:
            self.get_logger().warning(
                f'Ignoring target in frame {point.header.frame_id!r}; '
                f'expected {target_frame!r}.',
                throttle_duration_sec=2.0)
            return
        if not self.target_is_safe(point.point):
            self.get_logger().warning(
                'Ignoring non-finite or out-of-workspace target.',
                throttle_duration_sec=2.0)
            self.target_samples.clear()
            return
        stamp = point.header.stamp
        if stamp.sec or stamp.nanosec:
            age = (
                self.get_clock().now().nanoseconds -
                (stamp.sec * 1_000_000_000 + stamp.nanosec)
            ) / 1e9
            max_age = float(self.get_parameter('max_target_age').value)
            if age > max_age or age < -max_age:
                self.target_samples.clear()
                return
        tolerance = float(
            self.get_parameter('target_stability_tolerance').value)
        if (self.target_samples and self.point_distance(
                point.point, self.target_samples[-1]) > tolerance):
            self.target_samples.clear()
        self.target_samples.append(point.point)
        if len(self.target_samples) < self.target_samples.maxlen:
            return
        filtered_target = self.median_point(self.target_samples)
        if any(self.point_distance(sample, filtered_target) > tolerance
               for sample in self.target_samples):
            self.target_samples.clear()
            return
        try:
            current = self.tf_buffer.lookup_transform(
                target_frame, self.get_parameter('end_effector_link').value,
                Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warning(f'End-effector TF unavailable: {exc}')
            return

        self.target = filtered_target
        self.orientation = current.transform.rotation
        self.state = 'OPENING'
        self.get_logger().info(
            'Stable target acquired. Opening gripper before pre-grasp motion.')
        self.command_gripper(
            self.get_parameter('open_width').value, self.opened)

    def opened(self, succeeded):
        if not succeeded:
            self.fail('Could not open gripper')
            return
        self.state = 'MOVING_ABOVE'
        self.send_arm_goal(
            self.pose_goal(self.pose_at(
                self.get_parameter('approach_offset_z').value), linear=False),
            self.above_done)

    def above_done(self, succeeded):
        if not succeeded:
            self.fail('Could not reach the pre-grasp pose')
            return
        self.state = 'DESCENDING'
        self.send_arm_goal(
            self.pose_goal(self.pose_at(
                self.get_parameter('grasp_offset_z').value), linear=True),
            self.descended)

    def descended(self, succeeded):
        if not succeeded:
            self.fail('Could not reach the grasp pose')
            return
        self.state = 'CLOSING'
        self.command_gripper(
            self.get_parameter('closed_width').value, self.closed)

    def closed(self, succeeded):
        if not succeeded:
            self.fail('Could not close gripper')
            return
        # Wait for joint_states to catch the final controller sample before
        # checking that both fingers actually closed together.
        self.settle_timer = self.create_timer(
            self.get_parameter('grasp_settle_time').value,
            self.finish_close)

    def finish_close(self):
        if self.settle_timer is not None:
            self.settle_timer.cancel()
            self.settle_timer = None
        if not self.fingers_reached_goal(
                float(self.get_parameter('closed_width').value)):
            self.get_logger().warning(
                'Finger positions are not synchronized at the grasp; '
                'continuing with the Gazebo attachment fallback.')
        if not self.get_parameter('use_sim_attachment').value:
            self.begin_lift()
            return
        self.state = 'ATTACHING'
        # Require a new positive acknowledgement for this attach request; do
        # not accept a stale initial "attached" sample from Gazebo startup.
        self.attachment_state_received = False
        self.object_attached = False
        self.attachment_started_ns = self.get_clock().now().nanoseconds
        self.attach_publisher.publish(Empty())
        self.attachment_timer = self.create_timer(
            0.1, self.wait_for_attachment)

    def wait_for_attachment(self):
        if self.object_attached:
            self.attachment_timer.cancel()
            self.attachment_timer = None
            self.get_logger().info(
                'Both fingers closed; Gazebo object attachment confirmed.')
            self.begin_lift()
            return
        elapsed = (
            self.get_clock().now().nanoseconds - self.attachment_started_ns
        ) / 1e9
        if elapsed >= float(self.get_parameter('attachment_timeout').value):
            self.attachment_timer.cancel()
            self.attachment_timer = None
            self.get_logger().warning(
                'Gazebo did not publish attachment confirmation; continuing '
                'after repeated attach commands.')
            self.begin_lift()
            return
        self.attach_publisher.publish(Empty())

    def begin_lift(self):
        if self.settle_timer is not None:
            self.settle_timer.cancel()
            self.settle_timer = None
        self.state = 'LIFTING'
        self.send_arm_goal(
            self.pose_goal(
                self.pose_at(self.get_parameter('lift_offset_z').value),
                linear=True),
            self.lifted)

    def lifted(self, succeeded):
        if succeeded:
            self.state = 'DONE'
            self.get_logger().info('Pick sequence completed.')
        else:
            self.fail('Could not lift the object')

    def pose_at(self, z_offset):
        pose = Pose()
        pose.position.x = self.target.x
        pose.position.y = self.target.y
        pose.position.z = self.target.z + z_offset
        pose.orientation = self.orientation
        return pose

    def joint_goal(self):
        constraints = Constraints()
        for name, position in zip(self.JOINT_NAMES, self.POSE_1):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        return self.make_goal(
            constraints,
            self.get_parameter('point_to_point_planner').value)

    def pose_goal(self, pose, linear):
        link = self.get_parameter('end_effector_link').value
        tolerance = self.get_parameter('position_tolerance').value
        position = PositionConstraint()
        position.header.frame_id = self.get_parameter('target_frame').value
        position.link_name = link
        position.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tolerance * 2.0] * 3
        position.constraint_region.primitives.append(box)
        position.constraint_region.primitive_poses.append(pose)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.get_parameter('target_frame').value
        orientation.link_name = link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = self.get_parameter(
            'orientation_tolerance').value
        orientation.absolute_y_axis_tolerance = self.get_parameter(
            'orientation_tolerance').value
        orientation.absolute_z_axis_tolerance = self.get_parameter(
            'orientation_tolerance').value
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        planner = self.get_parameter(
            'linear_planner' if linear else 'point_to_point_planner').value
        return self.make_goal(constraints, planner)

    def make_goal(self, constraints, planner_id):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter('group_name').value
        goal.request.pipeline_id = self.get_parameter(
            'planning_pipeline').value
        goal.request.planner_id = planner_id
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.get_parameter(
            'velocity_scaling').value
        goal.request.max_acceleration_scaling_factor = self.get_parameter(
            'acceleration_scaling').value
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = not self.get_parameter(
            'execute_motion').value
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return goal

    def send_arm_goal(self, goal, done_callback):
        def result_ok(response):
            code = response.result.error_code.val
            return code == MoveItErrorCodes.SUCCESS, f'MoveIt error code {code}'

        self.send_action_goal(
            self.move_client, goal, result_ok, 'arm motion', done_callback)

    def command_gripper(self, width, done_callback):
        if not self.get_parameter('execute_motion').value:
            done_callback(True)
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.time_from_start = Duration(
            seconds=self.get_parameter('gripper_motion_time').value).to_msg()
        goal.trajectory.points = [point]
        self.send_gripper_goal(goal, done_callback)

    def joint_state_callback(self, message):
        positions = dict(zip(message.name, message.position))
        for name in ('left_finger_joint', 'right_finger_joint'):
            if name in positions:
                self.finger_positions[name] = positions[name]

    def attachment_state_callback(self, message):
        state = message.data.strip().lower()
        if state not in ('attached', 'detached'):
            self.get_logger().warning(
                f'Ignoring unknown Gazebo attachment state: {message.data!r}')
            return
        self.attachment_state_received = True
        self.object_attached = state == 'attached'
        if (state == 'detached' and
                self.startup_detach_timer is not None):
            self.startup_detach_timer.cancel()
            self.startup_detach_timer = None

    def publish_startup_detach(self):
        """Detach at startup without making motion depend on a state reply."""
        if self.startup_detach_timer is None:
            return
        if self.startup_detach_attempts >= 12:
            self.startup_detach_timer.cancel()
            self.startup_detach_timer = None
            return
        self.detach_publisher.publish(Empty())
        self.startup_detach_attempts += 1

    def fingers_reached_goal(self, goal):
        left = self.finger_positions.get('left_finger_joint')
        right = self.finger_positions.get('right_finger_joint')
        if left is None or right is None:
            self.get_logger().error('Finger joint states are unavailable')
            return False
        sync_tolerance = float(
            self.get_parameter('finger_sync_tolerance').value)
        goal_tolerance = float(
            self.get_parameter('finger_goal_tolerance').value)
        self.get_logger().info(
            f'Finger positions: left={left:.4f}, right={right:.4f}')
        return (
            abs(left - right) <= sync_tolerance and
            abs(left - goal) <= goal_tolerance and
            abs(right - goal) <= goal_tolerance
        )

    def send_gripper_goal(self, goal, done_callback):
        """Send a gripper trajectory with the same timeout/error handling."""
        def result_ok(response):
            result = response.result
            detail = result.error_string or (
                f'controller error code {result.error_code}')
            return result.error_code == 0, detail

        self.send_action_goal(
            self.gripper_client, goal, result_ok, 'gripper motion',
            done_callback)

    def send_action_goal(
            self, client, goal, result_ok, label, done_callback):
        """Send one action goal and always resolve it exactly once."""
        completed = False
        goal_handle = None

        def finish(succeeded, detail=''):
            nonlocal completed
            if completed:
                return
            completed = True
            timeout_timer.cancel()
            if not succeeded and detail:
                self.get_logger().error(f'{label} failed: {detail}')
            done_callback(succeeded)

        def timed_out():
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            finish(False, 'action timed out')

        timeout_timer = self.create_timer(
            float(self.get_parameter('action_timeout').value), timed_out)

        try:
            send_future = client.send_goal_async(goal)
        except Exception as exc:  # ROS middleware failures are surfaced here.
            finish(False, str(exc))
            return

        def result_callback(result_future):
            try:
                response = result_future.result()
                succeeded, detail = result_ok(response)
            except Exception as exc:
                finish(False, str(exc))
                return
            finish(succeeded, '' if succeeded else detail)

        def accepted_callback(response_future):
            nonlocal goal_handle
            try:
                goal_handle = response_future.result()
            except Exception as exc:
                finish(False, str(exc))
                return
            if not goal_handle.accepted:
                finish(False, 'goal was rejected')
                return
            goal_handle.get_result_async().add_done_callback(result_callback)

        send_future.add_done_callback(accepted_callback)

    def fail(self, message):
        if self.settle_timer is not None:
            self.settle_timer.cancel()
            self.settle_timer = None
        if self.attachment_timer is not None:
            self.attachment_timer.cancel()
            self.attachment_timer = None
        if self.startup_detach_timer is not None:
            self.startup_detach_timer.cancel()
            self.startup_detach_timer = None
        if (self.detach_publisher is not None and self.object_attached):
            self.detach_publisher.publish(Empty())
        self.state = 'FAILED'
        self.get_logger().error(message)

    def target_is_safe(self, point):
        """Check that a detected point is finite and in the configured box."""
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return False
        return (
            self.get_parameter('workspace_min_x').value <= point.x <=
            self.get_parameter('workspace_max_x').value and
            self.get_parameter('workspace_min_y').value <= point.y <=
            self.get_parameter('workspace_max_y').value and
            self.get_parameter('workspace_min_z').value <= point.z <=
            self.get_parameter('workspace_max_z').value
        )

    def parameters_are_valid(self):
        """Reject unsafe or internally inconsistent motion parameters."""
        errors = []
        approach = float(self.get_parameter('approach_offset_z').value)
        grasp = float(self.get_parameter('grasp_offset_z').value)
        lift = float(self.get_parameter('lift_offset_z').value)
        if approach <= grasp:
            errors.append('approach_offset_z must exceed grasp_offset_z')
        if lift <= grasp:
            errors.append('lift_offset_z must exceed grasp_offset_z')
        minimum = float(self.get_parameter('gripper_min_position').value)
        maximum = float(self.get_parameter('gripper_max_position').value)
        opened = float(self.get_parameter('open_width').value)
        closed = float(self.get_parameter('closed_width').value)
        if not minimum <= opened < closed <= maximum:
            errors.append(
                'gripper positions must satisfy min <= open < close <= max')
        if int(self.get_parameter('required_stable_detections').value) < 1:
            errors.append('required_stable_detections must be at least one')
        if float(self.get_parameter('action_timeout').value) <= 0.0:
            errors.append('action_timeout must be positive')
        if float(self.get_parameter('attachment_timeout').value) <= 0.0:
            errors.append('attachment_timeout must be positive')
        if float(self.get_parameter('finger_sync_tolerance').value) < 0.0:
            errors.append('finger_sync_tolerance must be non-negative')
        if float(self.get_parameter('finger_goal_tolerance').value) < 0.0:
            errors.append('finger_goal_tolerance must be non-negative')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            scaling = float(self.get_parameter(name).value)
            if not 0.0 < scaling <= 1.0:
                errors.append(f'{name} must be in the interval (0, 1]')
        for axis in ('x', 'y', 'z'):
            low = self.get_parameter(f'workspace_min_{axis}').value
            high = self.get_parameter(f'workspace_max_{axis}').value
            if low >= high:
                errors.append(f'workspace_min_{axis} must be below its max')
        for error in errors:
            self.get_logger().error(error)
        return not errors

    @staticmethod
    def median_point(points):
        """Return a coordinate-wise median without retaining message headers."""
        result = Point()
        result.x = median(point.x for point in points)
        result.y = median(point.y for point in points)
        result.z = median(point.z for point in points)
        return result

    @staticmethod
    def point_distance(first, second):
        return math.sqrt(
            (first.x - second.x) ** 2 +
            (first.y - second.y) ** 2 +
            (first.z - second.z) ** 2)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = TargetMotion()
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
