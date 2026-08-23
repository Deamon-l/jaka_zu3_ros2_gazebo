"""Plan and execute a vision-gated top-down RGB-D pick in Gazebo."""

import math

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, Pose
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
from shape_msgs.msg import SolidPrimitive
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
        self.declare_parameter('end_effector_link', 'gripper_tcp')
        self.declare_parameter('target_topic', '/detected_target_point')
        self.declare_parameter('approach_offset_z', 0.15)
        self.declare_parameter('grasp_offset_z', 0.04)
        self.declare_parameter('lift_offset_z', 0.18)
        self.declare_parameter('open_width', 0.025)
        self.declare_parameter('closed_width', 0.0)
        self.declare_parameter('required_stable_detections', 5)
        self.declare_parameter('target_stability_tolerance', 0.015)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')
        self.state = 'WAIT_SERVER'
        self.target = None
        self.last_target = None
        self.stable_detections = 0
        self.orientation = None
        self.create_subscription(
            PointStamped,
            self.get_parameter('target_topic').value, self.target_callback, 10)
        self.create_timer(0.5, self.start)
        execute = self.get_parameter('execute_motion').value
        mode = 'EXECUTE' if execute else 'PLAN_ONLY'
        self.get_logger().info(
            f'Waiting for MoveIt and gripper. mode={mode}; '
            'the robot will observe first, then act only after a stable target.')

    def start(self):
        if self.state != 'WAIT_SERVER':
            return
        if not self.move_client.server_is_ready():
            return
        if self.get_parameter('execute_motion').value and not (
                self.gripper_client.server_is_ready()):
            return
        self.state = 'MOVING_TO_OBSERVE'
        self.send_arm_goal(self.joint_goal(), self.observation_done)

    def observation_done(self, succeeded):
        if not succeeded:
            self.fail('Could not plan or execute pose_1')
            return
        self.state = 'WAIT_TARGET'
        self.get_logger().info('Observation pose reached. Waiting for target.')

    def target_callback(self, point):
        if self.state != 'WAIT_TARGET' or point.header.frame_id != 'world':
            return
        if self.last_target is None or self.point_distance(
                point.point, self.last_target) > self.get_parameter(
                    'target_stability_tolerance').value:
            self.stable_detections = 1
        else:
            self.stable_detections += 1
        self.last_target = point.point
        if self.stable_detections < self.get_parameter(
                'required_stable_detections').value:
            return
        try:
            current = self.tf_buffer.lookup_transform(
                'world', self.get_parameter('end_effector_link').value, Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warning(f'End-effector TF unavailable: {exc}')
            return

        self.target = point.point
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
            self.pose_goal(self.pose_at(self.get_parameter('approach_offset_z').value)),
            self.above_done)

    def above_done(self, succeeded):
        if not succeeded:
            self.fail('Could not reach the pre-grasp pose')
            return
        self.state = 'DESCENDING'
        self.send_arm_goal(
            self.pose_goal(self.pose_at(self.get_parameter('grasp_offset_z').value)),
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
        self.state = 'LIFTING'
        self.send_arm_goal(
            self.pose_goal(self.pose_at(self.get_parameter('lift_offset_z').value)),
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
        return self.make_goal(constraints)

    def pose_goal(self, pose):
        link = self.get_parameter('end_effector_link').value
        tolerance = self.get_parameter('position_tolerance').value
        position = PositionConstraint()
        position.header.frame_id = 'world'
        position.link_name = link
        position.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tolerance * 2.0] * 3
        position.constraint_region.primitives.append(box)
        position.constraint_region.primitive_poses.append(pose)

        orientation = OrientationConstraint()
        orientation.header.frame_id = 'world'
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
        return self.make_goal(constraints)

    def make_goal(self, constraints):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter('group_name').value
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = not self.get_parameter(
            'execute_motion').value
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return goal

    def send_arm_goal(self, goal, done_callback):
        future = self.move_client.send_goal_async(goal)

        def accepted_callback(response):
            handle = response.result()
            if not handle.accepted:
                done_callback(False)
                return
            handle.get_result_async().add_done_callback(
                lambda result: done_callback(
                    result.result().result.error_code.val ==
                    MoveItErrorCodes.SUCCESS))

        future.add_done_callback(accepted_callback)

    def command_gripper(self, width, done_callback):
        if not self.get_parameter('execute_motion').value:
            done_callback(True)
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        goal.trajectory.points = [point]
        future = self.gripper_client.send_goal_async(goal)

        def accepted_callback(response):
            handle = response.result()
            if not handle.accepted:
                done_callback(False)
                return
            handle.get_result_async().add_done_callback(
                lambda result: done_callback(
                    result.result().result.error_code == 0))

        future.add_done_callback(accepted_callback)

    def fail(self, message):
        self.state = 'FAILED'
        self.get_logger().error(message)

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
