#!/usr/bin/env python3
"""
航点跟随节点 — 启动后自动按顺序导航到 YAML 配置的航点。

使用 Nav2 NavigateThroughPoses 动作，无需障碍物检测。
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """偏航角 → 四元数"""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        # --- 参数 ---
        self.declare_parameter('waypoints', [
            {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 2.5, 'y': 0.0, 'yaw': 0.0},
        ])
        self.declare_parameter('return_to_start', False)
        self.declare_parameter('return_x', 0.0)
        self.declare_parameter('return_y', 0.0)
        self.declare_parameter('return_yaw', 0.0)
        self.declare_parameter('wait_at_waypoint', 0.0)
        self.declare_parameter('auto_start_delay', 5.0)

        self.waypoints = self.get_parameter('waypoints').value
        self.return_to_start = self.get_parameter('return_to_start').value
        self.wait_at_waypoint = self.get_parameter('wait_at_waypoint').value
        self.auto_start_delay = self.get_parameter('auto_start_delay').value

        # --- 构建航点列表 ---
        self.poses = []
        for i, wp in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(float(wp.get('yaw', 0.0)))
            self.poses.append(pose)
            self.get_logger().info(f'  航点{i+1}: x={wp["x"]:.2f}, y={wp["y"]:.2f}, '
                                  f'yaw={wp.get("yaw",0.0):.2f}')

        if self.return_to_start:
            ret_wp = self.get_parameter('return_x').value
            ret_wp_y = self.get_parameter('return_y').value
            ret_wp_yaw = self.get_parameter('return_yaw').value
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(ret_wp)
            pose.pose.position.y = float(ret_wp_y)
            pose.pose.orientation = yaw_to_quaternion(float(ret_wp_yaw))
            self.poses.append(pose)
            self.get_logger().info(f'  返回起点: x={ret_wp:.2f}, y={ret_wp_y:.2f}')

        self.get_logger().info(f'共 {len(self.poses)} 个航点, '
                              f'{self.auto_start_delay}秒后自动开始')

        # --- 动作客户端 ---
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # 延时启动
        self._timer = self.create_timer(self.auto_start_delay, self._start_nav)

    def _start_nav(self):
        """等待 Nav2 就绪，发送航点序列。"""
        self._timer.cancel()

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateThroughPoses 动作服务器未就绪!')
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.poses
        goal_msg.behavior_tree = ''

        self.get_logger().info(f'🚀 发送 {len(self.poses)} 个航点...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        self._send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝!')
            return

        self.get_logger().info('目标已接受, 开始执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        remaining = fb.number_of_poses_remaining
        dist = fb.distance_remaining
        if remaining % 2 == 1 or remaining <= 1:  # 避免刷屏
            self.get_logger().info(f'  📍 剩余航点: {remaining}, 剩余距离: {dist:.2f}m')

    def _result_cb(self, future):
        result = future.result()
        if result.result is not None:
            self.get_logger().info('✅ 所有航点完成!')
        else:
            self.get_logger().error('❌ 航点导航失败')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
