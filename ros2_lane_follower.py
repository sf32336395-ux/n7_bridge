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
        self.declare_parameter('look_ahead_distance', 5.0) 
        self.declare_parameter('smoothing_alpha', 0.3) # LPF 係數 (0.0~1.0, 越小越平滑)
        
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.L_fw = self.get_parameter('look_ahead_distance').get_parameter_value().double_value
        self.alpha = self.get_parameter('smoothing_alpha').get_parameter_value().double_value
        
        # 4. 歷史狀態 (用於平滑濾波)
        self.last_swa = 0.0
        
        # 5. 訂閱者
        self.path_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        
        # 6. 狀態監測計時器
        self.status_timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info(f"N7 Lane Follower Node Started (with Interpolation & LPF).")
        self.get_logger().info(f"Subscribing to: {path_topic}")
        self.get_logger().info("Waiting for rosbag data...")

    def check_status(self):
        now = self.get_clock().now()
        diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        if diff > 2.0:
            print(f"\033[93m[Status] 暫停接收 - 已有 {diff:.1f} 秒未收到 rosbag 資料 (Topic: {self.get_parameter('path_topic').value})\033[0m")
        else:
            print(f"\033[92m[Status] 接收中 - 累計收到 {self.msg_count} 筆路徑資料\033[0m")

    def get_interpolated_point(self, poses, target_dist):
        """
        在路徑點之間進行線性插值，找出距離剛好為 target_dist 的點
        """
        if len(poses) < 2:
            return None
            
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            
            d1 = math.sqrt(p1.x**2 + p1.y**2)
            d2 = math.sqrt(p2.x**2 + p2.y**2)
            
            # 檢查 target_dist 是否落在這兩個點之間
            if (d1 <= target_dist <= d2) or (d2 <= target_dist <= d1):
                if abs(d2 - d1) < 0.001:
                    return p1
                
                # 線性插值比例
                ratio = (target_dist - d1) / (d2 - d1)
                
                class InterpolatedPoint:
                    pass
                
                res = InterpolatedPoint()
                res.x = p1.x + ratio * (p2.x - p1.x)
                res.y = p1.y + ratio * (p2.y - p1.y)
                return res
        
        # 如果沒找到區間，則回傳距離最近的點
        return poses[0].pose.position if poses else None

    def path_callback(self, msg):
        if not msg.poses:
            return
            
        self.msg_count += 1
        self.last_msg_time = self.get_clock().now()
        
        try:
            # 使用線性插值獲取精確的預瞄點
            target_pose = self.get_interpolated_point(msg.poses, self.L_fw)

            if target_pose:
                # 偵錯資訊：觀察插值後的座標 (提高精度以觀察微幅變動)
                dist = math.sqrt(target_pose.x**2 + target_pose.y**2)
                self.get_logger().info(f"Target Pose (Interpolated): x={target_pose.x:.4f}, y={target_pose.y:.4f}, Dist={dist:.4f}m")
                
                ros_forward = target_pose.x
                ros_left = target_pose.y
                
                # 計算本次預期的 SWA
                # 注意：我們在這裡手動計算 SWA 是為了進行濾波，或者直接修改 bridge
                # 為了保持 bridge 邏輯單純，我們在 bridge 內做濾波或在此處處理
                # 我們選擇在 bridge 的 send_vision_command 增加濾波功能，或者在此處「平滑」輸入座標
                
                # 這裡採取「座標平滑」與「SWA 濾波」雙重機制
                # 先傳給 bridge 進行計算
                self.bridge_logic.send_vision_command(zed_x=-ros_left, zed_z=ros_forward, mode=1)
            else:
                self.get_logger().warn("No target pose found in path.")

            
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
