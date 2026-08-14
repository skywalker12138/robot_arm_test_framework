"""Run a sequence of joint trajectory tests against a ROS 2 controller."""

from arm_test_runner.csv_report import create_output_directory, save_reports
from arm_test_runner.plot_results import generate_plots
from arm_test_runner.test_config import (
    ERROR_TOLERANCE,
    EXECUTION_TIMEOUT_SECONDS,
    FINAL_STATE_DELAY_SECONDS,
    JOINT_NAMES,
    NEXT_TEST_DELAY_SECONDS,
    TEST_CASES,
    TRAJECTORY_DURATION_SECONDS,
)
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryTestNode(Node):
    """Send joint targets, observe joint states, and evaluate each test."""

    def __init__(self):
        super().__init__('trajectory_test_node')

        self.joint_names = JOINT_NAMES
        self.test_cases = TEST_CASES
        self.error_tolerance = ERROR_TOLERANCE
        self.execution_timeout = EXECUTION_TIMEOUT_SECONDS

        self.current_test_index = 0
        self.target_positions = None
        self.actual_positions = {}
        self.test_results = []
        self.joint_result_rows = []
        self.trajectory_rows = []
        self.output_directory = create_output_directory()

        self.goal_handle = None
        self.controller_error_code = None
        self.result_received = False
        self.timed_out = False
        self.current_test_finished = False
        self.current_test_active = False
        self.test_start_nanoseconds = None
        self.all_tests_finished = False

        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        self.get_logger().info('测试节点已经启动')
        self.get_logger().info(f'结果目录：{self.output_directory}')
        self.start_timer = self.create_timer(1.0, self.start_tests)

    def joint_state_callback(self, message):
        """Store the newest position for every named joint."""
        self.actual_positions = dict(zip(message.name, message.position))
        self.record_trajectory_sample()

    def record_trajectory_sample(self):
        """Record measured positions while the current goal is executing."""
        if not self.current_test_active or self.test_start_nanoseconds is None:
            return
        if any(name not in self.actual_positions for name in self.joint_names):
            return

        now_nanoseconds = self.get_clock().now().nanoseconds
        elapsed_seconds = (
            now_nanoseconds - self.test_start_nanoseconds
        ) / 1_000_000_000.0
        test_number = self.current_test_index + 1

        for joint_name, target in zip(self.joint_names, self.target_positions):
            actual = self.actual_positions[joint_name]
            self.trajectory_rows.append({
                'test_number': test_number,
                'elapsed_seconds': elapsed_seconds,
                'joint_name': joint_name,
                'target_position': target,
                'actual_position': actual,
                'absolute_error': abs(target - actual),
            })

    def start_tests(self):
        """Connect to the controller and start the first test."""
        self.start_timer.cancel()
        self.get_logger().info('正在等待arm_controller的Action服务器……')

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('找不到轨迹控制器Action服务器')
            self.finish_all_tests()
            return

        self.get_logger().info(
            f'Action服务器已连接，共有{len(self.test_cases)}组测试'
        )
        self.run_current_test()

    def run_current_test(self):
        """Build and send the goal for the current test case."""
        if self.current_test_index >= len(self.test_cases):
            self.finish_all_tests()
            return

        self.target_positions = self.test_cases[self.current_test_index]
        self.goal_handle = None
        self.controller_error_code = None
        self.result_received = False
        self.timed_out = False
        self.current_test_finished = False
        self.current_test_active = False
        self.test_start_nanoseconds = None

        test_number = self.current_test_index + 1
        self.get_logger().info('')
        self.get_logger().info(
            f'========== 开始第{test_number}/{len(self.test_cases)}组测试 =========='
        )
        self.get_logger().info(f'目标关节角：{self.target_positions}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.time_from_start.sec = TRAJECTORY_DURATION_SECONDS
        goal.trajectory.points = [point]

        test_index = self.current_test_index
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, test_index)
        )

    def goal_response_callback(self, future, test_index):
        """Handle the controller's goal acceptance response."""
        if test_index != self.current_test_index:
            return
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('当前目标被控制器拒绝')
            self.record_failed_test('goal_rejected')
            return

        self.get_logger().info('轨迹目标已被接受，机械臂开始运动')
        self.test_start_nanoseconds = self.get_clock().now().nanoseconds
        self.current_test_active = True
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.result_callback(result, test_index)
        )
        self.timeout_timer = self.create_timer(
            self.execution_timeout, self.timeout_callback
        )

    def result_callback(self, future, test_index):
        """Handle the final trajectory result."""
        if (
            test_index != self.current_test_index
            or self.timed_out
            or self.current_test_finished
        ):
            return

        self.result_received = True
        self.timeout_timer.cancel()
        self.controller_error_code = future.result().result.error_code
        self.get_logger().info(
            f'控制器返回结果，错误码：{self.controller_error_code}'
        )
        self.evaluation_timer = self.create_timer(
            FINAL_STATE_DELAY_SECONDS, self.evaluate_current_test
        )

    def timeout_callback(self):
        """Cancel and fail a goal that did not finish in time."""
        self.timeout_timer.cancel()
        if self.result_received or self.current_test_finished:
            return

        self.timed_out = True
        self.get_logger().error(
            f'TIMEOUT：轨迹执行超过{self.execution_timeout:.1f}秒'
        )
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Record whether the controller accepted the cancel request."""
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info('控制器已接受取消请求')
        else:
            self.get_logger().warning('控制器未确认取消请求')
        self.record_failed_test('timeout')

    def evaluate_current_test(self):
        """Compare final measured positions with the current target."""
        self.evaluation_timer.cancel()
        if self.current_test_finished:
            return
        self.current_test_active = False

        missing_joints = [
            name for name in self.joint_names if name not in self.actual_positions
        ]
        if missing_joints:
            self.get_logger().error(f'缺少关节状态：{missing_joints}')
            self.record_failed_test('missing_joint_states')
            return

        errors = []
        test_number = self.current_test_index + 1
        self.get_logger().info(f'---------- 第{test_number}组结果 ----------')
        for joint_name, target in zip(self.joint_names, self.target_positions):
            actual = self.actual_positions[joint_name]
            error = abs(target - actual)
            errors.append(error)
            self.joint_result_rows.append({
                'test_number': test_number,
                'joint_name': joint_name,
                'target_position': target,
                'actual_position': actual,
                'absolute_error': error,
                'tolerance': self.error_tolerance,
                'joint_passed': error <= self.error_tolerance,
            })
            self.get_logger().info(
                f'{joint_name}: 目标={target:.4f}, 实际={actual:.4f}, '
                f'误差={error:.6f} rad'
            )

        average_error = sum(errors) / len(errors)
        maximum_error = max(errors)
        controller_success = self.controller_error_code == 0
        error_success = maximum_error <= self.error_tolerance
        passed = controller_success and error_success

        if not controller_success:
            reason = f'controller_error_{self.controller_error_code}'
        elif not error_success:
            reason = 'error_too_large'
        else:
            reason = 'success'

        self.test_results.append({
            'test_number': test_number,
            'passed': passed,
            'average_error': average_error,
            'maximum_error': maximum_error,
            'reason': reason,
        })
        self.current_test_finished = True
        log_result = self.get_logger().info if passed else self.get_logger().error
        log_result(f'第{test_number}组：{"PASS" if passed else "FAIL"}')
        self.get_logger().info(f'本组平均误差：{average_error:.6f} rad')
        self.get_logger().info(f'本组最大误差：{maximum_error:.6f} rad')
        self.schedule_next_test()

    def record_failed_test(self, reason):
        """Record a test that has no valid final error measurement."""
        if self.current_test_finished:
            return
        self.current_test_active = False
        test_number = self.current_test_index + 1
        self.current_test_finished = True
        self.test_results.append({
            'test_number': test_number,
            'passed': False,
            'average_error': None,
            'maximum_error': None,
            'reason': reason,
        })
        self.get_logger().error(f'第{test_number}组：FAIL（{reason}）')
        self.schedule_next_test()

    def schedule_next_test(self):
        """Move to the next case after a short controller settling delay."""
        self.current_test_index += 1
        if self.current_test_index >= len(self.test_cases):
            self.finish_all_tests()
            return
        self.next_test_timer = self.create_timer(
            NEXT_TEST_DELAY_SECONDS, self.start_next_test
        )

    def start_next_test(self):
        self.next_test_timer.cancel()
        self.run_current_test()

    def finish_all_tests(self):
        """Print aggregate metrics, save reports, and stop the node."""
        if self.all_tests_finished:
            return
        self.all_tests_finished = True

        total_count = len(self.test_results)
        passed_count = sum(result['passed'] for result in self.test_results)
        failed_count = total_count - passed_count
        success_rate = 100.0 * passed_count / total_count if total_count else 0.0
        average_errors = [
            result['average_error'] for result in self.test_results
            if result['average_error'] is not None
        ]
        maximum_errors = [
            result['maximum_error'] for result in self.test_results
            if result['maximum_error'] is not None
        ]
        overall_average_error = (
            sum(average_errors) / len(average_errors) if average_errors else 0.0
        )
        overall_maximum_error = max(maximum_errors) if maximum_errors else 0.0

        self.get_logger().info('')
        self.get_logger().info('========== 全部测试汇总 ==========')
        self.get_logger().info(f'测试总数：{total_count}')
        self.get_logger().info(f'通过数量：{passed_count}')
        self.get_logger().info(f'失败数量：{failed_count}')
        self.get_logger().info(f'成功率：{success_rate:.2f}%')
        self.get_logger().info(f'全部测试平均误差：{overall_average_error:.6f} rad')
        self.get_logger().info(f'全部测试最大误差：{overall_maximum_error:.6f} rad')
        if total_count and not failed_count:
            self.get_logger().info('总体判定：PASS')
        else:
            self.get_logger().error('总体判定：FAIL')

        save_reports(
            self.output_directory,
            self.test_results,
            self.joint_result_rows,
            self.trajectory_rows,
        )
        plot_files = generate_plots(self.output_directory)
        self.get_logger().info(f'已生成{len(plot_files)}张轨迹曲线')
        self.get_logger().info(f'所有测试文件位于：{self.output_directory}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
