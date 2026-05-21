import math
import time
import numpy as np

# 嘗試導入核心庫
try:
    from doipclient import DoIPClient
    from doipclient.connectors import DoIPClientUDSConnector
    from udsoncan.client import Client
    from udsoncan.services import *
    from FoxPi_write import FoxPiWriteDID
    from FoxPi_TP import FoxPiTP
    LIB_AVAILABLE = True
except ImportError as e:
    print(f"[Warning] Required libraries not found: {e}")
    print("[Warning] Bridge will run in MOCK mode.")
    LIB_AVAILABLE = False

# N7 連線參數 (通常固定)
DOIP_SERVER_IP = "192.168.1.10" # 請依實車調整
DOIP_LOGICAL_ADDRESS = 0x1000   # 請依實車調整

class N7ControlBridge:
    def __init__(self, target_speed_kmh=5.0):
        self.L = 2.92
        self.steering_ratio = 15.0
        self.fixed_speed = target_speed_kmh
        self.uds_client = None
        
        if LIB_AVAILABLE:
            try:
                # 1. 建立 DoIP 連線
                self.doip_client = DoIPClient(DOIP_SERVER_IP, DOIP_LOGICAL_ADDRESS, protocol_version=3)
                self.uds_connection = DoIPClientUDSConnector(self.doip_client)
                
                # 2. 建立 UDS Client (使用預設 config)
                self.uds_client = Client(self.uds_connection, request_timeout=4)
                self.uds_client.open()
                
                # 3. 建立 TP 維護連線
                self.tp = FoxPiTP(self.uds_client)
                self.tp.start()
                
                # 4. 建立控制類別
                self.car = FoxPiWriteDID(self.uds_client)
                print(f"[N7 Bridge] Connected to vehicle at {DOIP_SERVER_IP}")
            except Exception as e:
                print(f"[N7 Bridge] Initialization failed: {e}")
                LIB_AVAILABLE = False
        
        if not LIB_AVAILABLE:
            self.tp = None
            self.car = None

    def send_vision_command(self, zed_x, zed_z, mode=1):
        """
        zed_x: 左右偏移 (m)
        zed_z: 前方距離 (m)
        mode: 1=啟動控制, 0=停止
        """
        car_x = zed_z
        car_y = -zed_x
        ld = math.sqrt(car_x**2 + car_y**2)
        
        if ld < 1.0:
            swa = 0.0
        else:
            delta = math.atan(2 * self.L * car_y / (ld**2))
            swa = math.degrees(delta) * self.steering_ratio
        
        # N7 安全限制
        swa = np.clip(swa, -450, 450)
        
        # --- 封裝 14 個參數 (對應 FoxPi_Driving_Ctrl) ---
        # 根據 write.py 的 WID_param 定義：
        # 0: AccReq, 1: AccReq_A, 2: TargetSpd, 3: TargetSpd_A,
        # 4: Angle_V, 5: Angle_Req, 6: Angle (SWA), 
        # 7: Torque_V, 8: Torque_Req, 9: Torque, 
        # 10: APSVMCReqA_flg, 11: APSStaSystem, 12: APSShiftPosnReq, 13: APSSpeedCMD
        
        control_list = [
            0.5,      # 0: AccReq (加速度需求)
            1,        # 1: AccReq_A (啟用加速度控制)
            self.fixed_speed, # 2: TargetSpd (km/h)
            1,        # 3: TargetSpd_A (啟用速度控制)
            mode,     # 4: Angle_V (轉向有效位元)
            mode,     # 5: Angle_Req (轉向請求位元)
            swa,      # 6: Angle (方向盤轉角)
            0,        # 7: Torque_V
            0,        # 8: Torque_Req
            0,        # 9: Torque
            mode,     # 10: APSVMCReqA_flg (控制總開關)
            4,        # 11: APSStaSystem (系統狀態, 參考 default=4)
            7,        # 12: APSShiftPosnReq (檔位請求, 參考 default=7)
            self.fixed_speed # 13: APSSpeedCMD
        ]

        if LIB_AVAILABLE and self.car:
            self.car.FoxPi_Driving_Ctrl(control_list)
        
        print(f"[N7 Bridge] LD: {ld:.1f}m | SWA: {swa:.1f}° | Speed: {self.fixed_speed}km/h | Mode: {'ENABLE' if mode else 'STOP'}")

    def stop(self):
        if LIB_AVAILABLE:
            try:
                self.send_vision_command(0, 5, mode=0)
                if self.tp: self.tp.stop()
                if self.uds_client: self.uds_client.close()
            except:
                pass
        print("[N7 Bridge] Stopped and cleaned up.")
