#!/usr/bin/env python3
"""
ROS2 Humble 的 AprilTag 检测节点。
从 2025_robocon_code (ROS1 Noetic) 移植而来 — 使用 pupil_apriltags (纯 Python)。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

from robocon_interfaces.msg import TagDetection, TagDetectionArray


class AprilTagDetectorNode(Node):
    """从 RealSense D455 RGB 图像流中检测 AprilTag。"""

    def __init__(self):
        super().__init__('apriltag_detector')

        # 参数
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.16)           # 16cm 标签
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('visualize', True)

        tag_family = self.get_parameter('tag_family').value
        tag_size = self.get_parameter('tag_size').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.visualize = self.get_parameter('visualize').value

        # 相机内参 (将通过 CameraInfo 更新)
        self.fx = 495.18  # 默认值: D455 近似值
        self.fy = 495.46
        self.cx = 311.01
        self.cy = 221.22
        self.has_camera_info = False

        # 检测器
        self.detector = Detector(
            families=tag_family,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.tag_size = tag_size

        # CV Bridge
        self.bridge = CvBridge()

        # 订阅者
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # 发布者
        self.detection_pub = self.create_publisher(
            TagDetectionArray, '/tag_detections', 10)

        self.get_logger().info(
            f'AprilTag 检测器已初始化: family={tag_family}, size={tag_size}m')

    def camera_info_callback(self, msg: CameraInfo):
        """从 CameraInfo 更新相机内参。"""
        if self.has_camera_info:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.has_camera_info = True
        self.get_logger().info(
            f'相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}')

    def image_callback(self, msg: Image):
        """处理输入图像并检测 AprilTag。"""
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {e}')
            return

        # 检测标签并进行位姿估计
        camera_params = (self.fx, self.fy, self.cx, self.cy)
        detections = self.detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self.tag_size
        )

        # 构建检测数组
        tag_array = TagDetectionArray()
        tag_array.header = msg.header

        for det in detections:
            td = TagDetection()
            td.tag_id = det.tag_id
            td.confidence = det.decision_margin

            # 位姿 (相机光心坐标系)
            td.pose.position.x = float(det.pose_t[0])
            td.pose.position.y = float(det.pose_t[1])
            td.pose.position.z = float(det.pose_t[2])
            td.distance = float(np.linalg.norm(det.pose_t))

            # 旋转矩阵 -> 四元数
            R = det.pose_R
            # 将旋转矩阵转换为四元数
            import math
            trace = R[0,0] + R[1,1] + R[2,2]
            if trace > 0:
                s = 0.5 / math.sqrt(trace + 1.0)
                w = 0.25 / s
                x = (R[2,1] - R[1,2]) * s
                y = (R[0,2] - R[2,0]) * s
                z = (R[1,0] - R[0,1]) * s
            elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                w = (R[2,1] - R[1,2]) / s
                x = 0.25 * s
                y = (R[0,1] + R[1,0]) / s
                z = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                w = (R[0,2] - R[2,0]) / s
                x = (R[0,1] + R[1,0]) / s
                y = 0.25 * s
                z = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                w = (R[1,0] - R[0,1]) / s
                x = (R[0,2] + R[2,0]) / s
                y = (R[1,2] + R[2,1]) / s
                z = 0.25 * s

            td.pose.orientation.w = float(w)
            td.pose.orientation.x = float(x)
            td.pose.orientation.y = float(y)
            td.pose.orientation.z = float(z)

            tag_array.detections.append(td)

        self.detection_pub.publish(tag_array)

        # 可视化
        if self.visualize and detections:
            for det in detections:
                # 绘制边界框角点
                for i in range(4):
                    pt1 = tuple(map(int, det.corners[i]))
                    pt2 = tuple(map(int, det.corners[(i+1)%4]))
                    cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)

                # 绘制标签 ID 和距离
                cx, cy = map(int, det.center)
                cv2.putText(cv_image, f'ID:{det.tag_id} d:{td.distance:.2f}m',
                            (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('AprilTag 检测', cv_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
