import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import sys
import os

# 嘗試加入 bridge 路徑以引用 N7ControlBridge
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from n7_control_bridge import N7ControlBridge

class N7LaneFollower(Node):
    def __init__(self):
        super().__init__('n7_lane_follower')
        
        # 1. 初始化控制橋接器
        self.bridge_logic = N7ControlBridge(target_speed_kmh=5.0)
        
        # 2. 狀態追蹤
        self.msg_count = 0
        self.last_msg_time = self.get_clock().now()
        
        # 3. 參數設定
        self.declare_parameter('path_topic', '/centerline')
        self.declare_parameter('look_ahead_distance', 5.0) # 預設拉長到 5m 增加穩定性
        
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.L_fw = self.get_parameter('look_ahead_distance').get_parameter_value().double_value
        
        # 4. 訂閱者
        self.path_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        
        # 5. 狀態監測計時器 (每 2 秒檢查一次是否收到資料)
        self.status_timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info(f"N7 Lane Follower Node Started.")
        self.get_logger().info(f"Subscribing to: {path_topic}")
        self.get_logger().info("Waiting for rosbag data...")

    def check_status(self):
        now = self.get_clock().now()
        diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        if diff > 2.0:
            print(f"\033[93m[Status] 暫停接收 - 已有 {diff:.1f} 秒未收到 rosbag 資料 (Topic: {self.get_parameter('path_topic').value})\033[0m")
        else:
            print(f"\033[92m[Status] 接收中 - 累計收到 {self.msg_count} 筆路徑資料\033[0m")

    def path_callback(self, msg):
        if not msg.poses:
            return
            
        self.msg_count += 1
        self.last_msg_time = self.get_clock().now()
        
        try:
            # 尋找距離車輛最近且最接近 L_fw (預瞄距離) 的點
            # ROS Path 通常是以車體或相機為原點
            target_pose = None
            min_dist_error = float('inf')
            
            for pose_stamped in msg.poses:
                px = pose_stamped.pose.position.x
                py = pose_stamped.pose.position.y
                pz = pose_stamped.pose.position.z
                
                # 計算該點距離原點的 2D 距離 (假設原點是相機)
                # 注意：根據 ZED ROS 2 慣例，前方通常是 X 或 Z
                # 從 n7_control_bridge 來看，它期待 zed_x (左右) 和 zed_z (前方)
                dist = math.sqrt(px**2 + py**2 + pz**2)
                
                error = abs(dist - self.L_fw)
                if error < min_dist_error:
                    min_dist_error = error
                    target_pose = pose_stamped.pose.position

            if target_pose:
                # 偵錯資訊：觀察原始座標
                # px: X, py: Y, pz: Z
                self.get_logger().info(f"Target Pose (Raw): x={target_pose.x:.2f}, y={target_pose.y:.2f}, z={target_pose.z:.2f}")
                
                # ZED ROS2 慣例: X 是前方, Y 是左方, Z 是上方
                # 但 n7_control_bridge.py 的 send_vision_command(zed_x, zed_z) 
                # 內部邏輯是: car_x = zed_z, car_y = -zed_x
                # 如果 ROS Topic 裡的 X 是前方，我們應該這樣傳：
                # 這裡我們先嘗試把 ROS_X 當作 zed_z (前方)，把 ROS_Y 當作 zed_x (左右)
                ros_forward = target_pose.x
                ros_left = target_pose.y
                
                # 傳給 bridge: zed_x (左右), zed_z (前方)
                self.bridge_logic.send_vision_command(zed_x=-ros_left, zed_z=ros_forward, mode=1)
            else:
                self.get_logger().warn("No target pose found in path within look-ahead range.")
            
        except Exception as e:
            self.get_logger().error(f"Error in path_callback: {str(e)}")

    def stop(self):
        self.bridge_logic.stop()

def main(args=None):
    rclpy.init(args=args)
    node = N7LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


    def stop(self):
        self.bridge_logic.stop()

def main(args=None):
    rclpy.init(args=args)
    node = N7LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
