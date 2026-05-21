"""
整合範例：如何將 zed_yolo_tf 與 n7_control_bridge 接合。
"""
import sys
import time
# 確保能導入 bridge 檔案
from n7_control_bridge import N7ControlBridge

def main_loop():
    # 1. 初始化橋接器
    # 建議先用模擬模式測試，或在空地測試
    bridge = N7ControlBridge(target_speed_kmh=5.0)

    try:
        print("啟動 N7 自動駕駛整合循環...")
        
        # 這裡模擬 zed_yolo_tf 的偵測循環
        while True:
            # --- 步驟 A: 從 ZED 獲取中心線點 ---
            # 在實際程式中，這部分會是 zed_yolo_tf 的偵測輸出
            # 假設我們得到了一個目標點 target_point = [x, y, z] (公尺)
            # 例如：x = 0.5 (偏右), z = 4.0 (前方 4 公尺)
            mock_target = [0.5, 0.0, 4.0] 
            
            # --- 步驟 B: 透過橋接器發送控制指令 ---
            bridge.send_vision_command(mock_target[0], mock_target[2])
            
            # 控制頻率建議在 10Hz ~ 20Hz
            time.sleep(0.1) 
            
    except KeyboardInterrupt:
        print("\n使用者中斷，正在安全停止...")
    finally:
        # 3. 務必執行 stop 以釋放車輛控制權
        bridge.stop()

if __name__ == "__main__":
    main_loop()
