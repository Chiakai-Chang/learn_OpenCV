# -*- coding: utf-8 -*-
"""
Created on Wed Dec  6 11:19:53 2023

@author: Chiakai
"""

import serial
import time
import cv2
import traceback

class servo_control:
    
    # Pin 
    Pin_up_down = 1      # 0 ~ 180
    Pin_left_right = 2   # 0 ~ 180
    Pin_on_off = 3       # 70 == on, 180 == off

    # 紀錄 servo 目前數值
    servo_ud_value = 90
    servo_lr_value = 90
    servo_onoff_value = 180
    
    # 與鏡頭畫面互動
    mouse_xy = [] # 當前滑鼠座標位置
    mouse_event = None 
    
    def __init__(
            self, 
            # Serial 要通訊的 COM_PORT
            COM_PORT,
            Baud_rate = 115200,
            # 鏡頭編號
            camera_id = 1,
            # 移動的比率
            move_rate = 1,
            ):
        
        self.COM_PORT = COM_PORT
        self.Baud_rate = Baud_rate
        self.camera_id = camera_id
        
        # CV2 的鏡頭物件
        print('>>啟動中，請稍候...')
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(">>Cannot open camera")
            input('------ Enter to Exit ------')
            exit()
        
        # 取得影像的尺寸大小
        self.video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f">>Camera Image Size: {self.video_width}, {self.video_height}")
        
        # 畫面中心點
        self.center_ud =  self.video_height / 2
        self.center_lr =  self.video_width / 2
        
        # 取得影像尺寸與 0 ~ 180 之間的 ratio
        self.ud_ratio = 180 / self.video_height
        self.lr_ratio = 180 / self.video_width
        
        # 移動的倍率(好像用不到)
        self.move_rate = move_rate
        
        # Serial 溝通物件
        self.controller = serial.Serial(self.COM_PORT, self.Baud_rate, timeout=1)
        
        print('>>Servo 已啟動...')
        self.reset_servo()
        
        # 追蹤用物件
        # 特別註記
        # 以下的 TrackerCSRT_create 必須要以下版本才能用
        # pip install opencv-contrib-python==4.5.1.48
        self.tracker = cv2.TrackerCSRT_create()  # 創建追蹤器
        self.tracking = False                    # 設定 False 表示尚未開始追蹤
        
        self.run()
        print('>>已開啟鏡頭視窗...')        
        
    def reset_servo(self):
        # 將攝影機 servo 置中
        self.servo_ud_value = 90
        self.servo_lr_value = 90
        self.servo_onoff_value = 180
        
        self.move_servo(self.Pin_up_down, self.servo_ud_value)
        self.move_servo(self.Pin_left_right, self.servo_lr_value)
        self.move_servo(self.Pin_on_off, self.servo_onoff_value)
        print('>>Servo 位置已歸零...')
        
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
    
    def mouse_action(self, event, x, y, flags, frame):
        self.mouse_xy = [x, y]
        self.mouse_event = [event, x, y, flags, frame]
        
        if event == cv2.EVENT_LBUTTONDOWN:
           print(f'Event: {event}')
    
    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Cannot receive frame")
                    break
                
                cv2.setMouseCallback('Remote Guard', self.mouse_action, frame)
                
                try:
                    cv2.putText(frame, f'X={self.mouse_xy[0]}, Y={self.mouse_xy[1]}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                except:
                    pass
                #frame = cv2.resize(frame,(540,300))  # 縮小尺寸，加快速度
                # 將按鈕說明寫在圖上
                cv2.putText(frame, 'Press "q" to quit...', (10, int(self.video_height-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)

                cv2.imshow('Remote Guard', frame)
                keyName = cv2.waitKey(1)  & 0xFF
                
                if keyName == ord('q'):
                    break
                if keyName == ord('a'):
                    area = cv2.selectROI('Remote Guard', frame, showCrosshair=False, fromCenter=False)
                    self.tracker.init(frame, area)    # 初始化追蹤器
                    self.tracking = True              # 設定可以開始追蹤
                        

            self.cap.release()
            cv2.destroyAllWindows()
        except:
            x = traceback.format_exc()
            print(x)

if __name__ == '__main__':
    controller = servo_control('COM4')


