# -*- coding: utf-8 -*-
"""
Created on Wed Dec  6 11:19:53 2023

@author: Chiakai
"""

import serial
import time
import cv2
import traceback

tracker = cv2.TrackerCSRT_create()  # 創建追蹤器
tracking = False                    # 設定 False 表示尚未開始追蹤

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

class servo_control:
    
    # Pin 
    Pin_up_down = 1      # 0 ~ 180
    Pin_left_right = 2   # 0 ~ 180
    Pin_on_off = 3       # 70 == on, 180 == off

    # 紀錄 servo 目前數值
    servo_ud_value = 90
    servo_lr_value = 90
    servo_onoff_value = 180
    
    def __init__(
            self, 
            # CV2 的鏡頭物件
            cap,
            # Serial 要通訊的 COM_PORT
            COM_PORT,
            Baud_rate = 115200,
            # 移動的比率
            move_rate = 1,
            ):
        
        self.cap = cap
        self.COM_PORT = COM_PORT
        self.Baud_rate = Baud_rate
        
        # 取得影像的尺寸大小
        self.video_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.video_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Camera Image Size: {self.video_width}, {self.video_height}")
        
        # 畫面中心點
        self.center_ud =  self.video_height / 2
        self.center_lr =  self.video_width / 2
        
        # 取得影像尺寸與 0 ~ 180 之間的 ratio
        self.ud_ratio = self.video_width / 180
        self.lr_ratio = self.video_width / 180
        
        # 移動的倍率(好像用不到)
        self.move_rate = move_rate
        
        # Serial 溝通物件
        self.controller = serial.Serial(self.COM_PORT, self.Baud_rate, timeout=1)
        
        print('>>Servo 已啟動...')
        self.reset_servo()
        print('>>Servo 位置已歸零...')
        
    def reset_servo(self):
        # 將攝影機 servo 置中
        self.servo_ud_value = 90
        self.servo_lr_value = 90
        self.servo_onoff_value = 180
        
        self.move_servo(self.Pin_up_down, self.servo_ud_value)
        self.move_servo(self.Pin_left_right, self.servo_lr_value)
        self.move_servo(self.Pin_on_off, self.servo_onoff_value)
        
    def move_servo(self, servo_number, position):
        command = f"{servo_number} {position}\n"
        print(f'command = {command}')
        self.controller.write(command.encode())
            

    def move_to(
        self,
        # 目標中心位置
        track_ud,
        track_lr,        
        ):
        self.track_ud = track_ud
        self.track_lr = track_lr
        
        # 將畫面按比例切割，
        
        #print(f'>>[servo移動] x = {new_lr}, y = {new_ud}')
        #input()
        #return new_lr, new_ud

mouse_xy = []
def mouse_action(event, x, y, flags, frame):
    global mouse_xy
    mouse_xy = [x, y]

p1 = None
p2 = None

# 取得影像的尺寸大小
video_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
video_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Cannot receive frame")
            break
        
        cv2.setMouseCallback('Remote Guard', mouse_action, frame)
        
        try:
            cv2.putText(frame, f'X={mouse_xy[0]}, Y={mouse_xy[1]}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
        except:
            pass
        #frame = cv2.resize(frame,(540,300))  # 縮小尺寸，加快速度
        # 將按鈕說明寫在圖上
        cv2.putText(frame, 'Press "q" to quit...', (10, int(video_height-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)

        cv2.imshow('Remote Guard', frame)
        keyName = cv2.waitKey(1)  & 0xFF
        
        if keyName == ord('q'):
            break
        if keyName == ord('a'):
            area = cv2.selectROI('Remote Guard', frame, showCrosshair=False, fromCenter=False)
            tracker.init(frame, area)    # 初始化追蹤器
            tracking = True              # 設定可以開始追蹤
                

    cap.release()
    cv2.destroyAllWindows()
except:
    x = traceback.format_exc()
    print(x)
