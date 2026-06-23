#!/usr/bin/env python3
"""Standalone trajectory executor — replaces JTC for Humble bug workaround.

Listens for FollowJointTrajectory action goals (from MoveIt2),
interpolates trajectory points, and publishes position commands
to /position_commands.  The DmMotorHardware plugin subscribes to
that topic and sends CAN frames to motors.

Why: Humble rclcpp lifecycle sub-nodes don't expose functional
parameter services, so the standard JTC can never receive its
joints/command_interfaces params.
"""
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


NUM_JOINTS = 6
PUBLISH_RATE = 100.0  # Hz — matches CM update rate


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._cmd_pub = self.create_publisher(
            Float64MultiArray, '/position_commands', 10
        )

        self._lock = threading.Lock()
        self._active_traj = None
        self._active = False
        self._period = 1.0 / PUBLISH_RATE

        self.get_logger().info(
            'TrajectoryExecutor ready — action: '
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def execute_callback(self, goal_handle):
        """Execute a trajectory — called in a separate thread by rclpy."""
        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names
        points = traj.points

        self.get_logger().info(
            f'Received trajectory: {len(points)} points for {joint_names}'
        )

        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # Validate joint count
        if len(joint_names) != NUM_JOINTS:
            self.get_logger().error(
                f'Expected {NUM_JOINTS} joints, got {len(joint_names)}'
            )
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Both MoveIt trajectory joints and the hardware driver's
        # /position_commands use the URDF joint order [joint1..joint6].
        # No remapping is needed — the JSB output order in /joint_states
        # is a presentation detail that MoveIt handles via name-based
        # matching, and does not affect command routing.

        # Store active trajectory
        with self._lock:
            self._active_traj = {
                'points': points,
                'start_time': self.get_clock().now(),
            }
            self._active = True

        # Publish trajectory points at the right times
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        dt = self._period
        point_idx = 0
        last_publish_time = self.get_clock().now()
        cmd_msg = Float64MultiArray()

        while rclpy.ok() and self._active:
            now = self.get_clock().now()
            elapsed = (now - self._active_traj['start_time']).nanoseconds * 1e-9

            # Find the current segment
            while point_idx < len(points) - 1:
                from_time = (points[point_idx].time_from_start.sec +
                             points[point_idx].time_from_start.nanosec * 1e-9)
                to_time = (points[point_idx + 1].time_from_start.sec +
                           points[point_idx + 1].time_from_start.nanosec * 1e-9)
                if to_time < elapsed:
                    point_idx += 1
                else:
                    break

            # Check if we've reached the end
            last_point_time = (points[-1].time_from_start.sec +
                               points[-1].time_from_start.nanosec * 1e-9)
            if elapsed >= last_point_time:
                # Send final position
                cmd_msg.data = list(points[-1].positions)
                self._cmd_pub.publish(cmd_msg)
                break

            # Interpolate between points
            if point_idx < len(points) - 1:
                from_pt = points[point_idx]
                to_pt = points[point_idx + 1]
                from_t = (from_pt.time_from_start.sec +
                          from_pt.time_from_start.nanosec * 1e-9)
                to_t = (to_pt.time_from_start.sec +
                        to_pt.time_from_start.nanosec * 1e-9)
                seg_dur = to_t - from_t

                if seg_dur > 0:
                    frac = (elapsed - from_t) / seg_dur
                    frac = max(0.0, min(1.0, frac))
                    positions = [
                        from_pt.positions[j] + frac * (to_pt.positions[j] - from_pt.positions[j])
                        for j in range(NUM_JOINTS)
                    ]
                else:
                    positions = list(to_pt.positions)
            else:
                positions = list(points[point_idx].positions)

            # Publish positions in MoveIt/URDF joint order
            cmd_msg.data = positions
            self._cmd_pub.publish(cmd_msg)

            # Publish feedback at ~50 Hz
            if (now - last_publish_time).nanoseconds * 1e-9 > 0.02:
                fb = JointTrajectoryPoint()
                fb.positions = positions  # feedback in MoveIt's order
                fb.time_from_start.sec = int(elapsed)
                fb.time_from_start.nanosec = int((elapsed - int(elapsed)) * 1e9)
                feedback.desired = fb
                feedback.actual = fb  # in dry-run, desired=actual
                goal_handle.publish_feedback(feedback)
                last_publish_time = now

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trajectory cancelled')
                goal_handle.canceled()
                with self._lock:
                    self._active = False
                return result

            time.sleep(dt)

        # Clean up
        with self._lock:
            self._active = False
            self._active_traj = None

        self.get_logger().info('Trajectory complete')
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    executor = TrajectoryExecutor()
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
