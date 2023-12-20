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
    
    # 追蹤器名稱
    name = ''
    
    # 版本號
    version = '20231220_0'
    
    # Pin 
    Pin_up_down = 1      # 0 ~ 180
    Pin_left_right = 2   # 0 ~ 180
    Pin_on_off = 3       # 70 == on, 180 == off

    # 紀錄 servo 目前數值
    current_ud_value = 90
    current_lr_value = 90
    current_onoff_value = 180
    
    # Tracking Box 左上、右下座標
    frame = None
    tracking = False
    tracking_left_up = [-1, -1]
    tracking_right_down = [-1, -1]
    tracking_ud = -1 # Tracking Box 中央 y
    tracking_lr = -1 # Tracking Box 中央 x
    
    # 若誤差不大就暫不移動
    allow_diff = 2
    
    # 若太久就自動 Reset
    idle_start = 0
    idle_count = 0
    # 當 light off，且
    # idle_count > idle_to_reset 時 Reset
    idle_to_reset = 5 # 秒
    
    # 與鏡頭畫面互動
    mouse_xy = [] # 當前滑鼠座標位置
    roi_wait_start = -1  # 按下滑鼠以後，開始等待
    roi_wait_count = 0   # 累積等待多久
    roi_wait_limit = 1.5 # 按下滑鼠到放開多久才算
    roi_click_down = []  # 儲存可能的 BBox 第一個點位
    roi_click_up   = []  # 儲存可能的 BBox 第二個點位
    # 會從前兩個算出左上右下
    roi_bbox_left_up = [] 
    roi_bbox_right_down = []
    
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
        
        # 創建追蹤器
        # 特別註記
        # 以下的 TrackerCSRT_create 必須要以下版本才能用
        # pip install opencv-contrib-python==4.5.1.48
        self.tracker = cv2.TrackerCSRT_create()  
        
        self.run()
        print('>>已開啟鏡頭視窗...')        
        
    def reset_servo(self):
        # 將攝影機 servo 置中
        self.current_ud_value = 90
        self.current_lr_value = 90
        self.current_onoff_value = 180
        
        for p, v in [
                (self.Pin_up_down, self.current_ud_value),
                (self.Pin_left_right, self.current_lr_value),
                (self.Pin_on_off, self.current_onoff_value),
                ]:
            command = f"{p} {v}\n"
            print(f'command = {command}')
            self.controller.write(command.encode())
        print('>>Servo 位置已歸零...')
    
    def light_on(self):
        self.servo_onoff_value = 70
        command = f"{self.Pin_on_off} {self.servo_onoff_value}\n"
        print(f'command = {command}')
        self.controller.write(command.encode())
    
    def light_off(self):
        self.current_onoff_value = 180
        command = f"{self.Pin_on_off} {self.current_onoff_value}\n"
        print(f'command = {command}')
        self.controller.write(command.encode())
    
    def move_updown(self, value):
        move_value = value * self.ud_ratio
        if abs(move_value) > self.allow_diff:
            new_ud_value = self.current_ud_value + move_value
            if 0 <= new_ud_value <= 180:
                self.current_ud_value = new_ud_value
                command = f"{self.Pin_up_down} {self.current_ud_value}\n"
                print(f'command = {command}')
                self.controller.write(command.encode())
            else:
                print(f'[Limit] Servo ud 想移動到: {new_ud_value}, 已超出可移動範圍...')
        else:
            print(f'[Limit] Servo ud 想微微移動: {move_value} 的量, 移動幅度太小，略...')
                
    def move_leftright(self, value):
        move_value = value * self.lr_ratio
        if abs(move_value) > self.allow_diff:
            new_lr_value = self.current_lr_value + move_value
            if 0 <= new_lr_value <= 180:
                self.current_lr_value = new_lr_value
                command = f"{self.Pin_left_right} {self.current_lr_value}\n"
                print(f'command = {command}')
                self.controller.write(command.encode())
            else:
                print(f'[Limit] Servo lr 想移動到: {new_lr_value}, 已超出可移動範圍...')
        else:
            print(f'[Limit] Servo lr 想微微移動: {move_value} 的量, 移動幅度太小，略...')
    
    def mouse_action(self, event, x, y, flags, frame):
        self.frame = frame
        self.mouse_xy = [x, y]
        self.mouse_event = [event, x, y, flags, frame]
        
        if event == cv2.EVENT_LBUTTONDOWN:
           print(f'>>Event: {event}')
           print(f'Position X: {x}')
           print(f'Position Y: {y}')
           
           # 計算可能要畫 ROI BBox
           self.roi_wait_start = time.time()
           self.roi_click_down = [x, y]
           
           # 如果開啟著 Tracking 點 Box 就關閉 Tracking
           if self.tracking:
               if self.tracking_left_up[0] <= x <= self.tracking_right_down[0]:
                   if self.tracking_left_up[1] <= y <= self.tracking_right_down[1]:
                       print('>>停止追蹤...')
                       self.tracking = False
                       self.light_off()
        
        if event == cv2.EVENT_LBUTTONUP:
            # 計算是否是在畫 ROI BBox
            self.roi_wait_count = time.time() - self.roi_wait_start
            if self.roi_wait_count >= self.roi_wait_limit:
                self.roi_click_up = [x, y]
                
                # 計算左上右下
                left  = min([self.roi_click_down[0], self.roi_click_up[0]])
                right = max([self.roi_click_down[0], self.roi_click_up[0]])
                up    = min([self.roi_click_down[1], self.roi_click_up[1]])
                down  = max([self.roi_click_down[1], self.roi_click_up[1]])
                
                self.roi_bbox_left_up    = [left,  up]
                self.roi_bbox_right_down = [right, down]
                print(f'self.roi_bbox_left_up = {self.roi_bbox_left_up}')
                print(f'self.roi_bbox_right_down = {self.roi_bbox_right_down}')
                area = cv2.selectROI(self.name, frame, showCrosshair=False, fromCenter=False)
                print(area)
                input()
                self.tracker.init(frame, area)    # 初始化追蹤器
                self.tracking = True              # 設定可以開始追蹤
                self.light_on()
    
    def count_diff(self):
        self.diff_ud =  self.tracking_ud - self.center_ud
        self.diff_lr =  self.center_lr - self.tracking_lr      
    
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
                    print(f'area = {area}')
                    self.tracker.init(frame, area)    # 初始化追蹤器
                    self.tracking = True              # 設定可以開始追蹤
                    self.light_on()
                        
                # 沒在追蹤也沒在中心位置太久，就恢復原位
                if not self.tracking:
                    if self.idle_count == 0:
                        self.idle_start = time.time()
                    else:
                        self.idle_count = time.time() - self.idle_start
                    
                    if self.idle_count > self.idle_to_reset:
                        if self.current_ud_value != 90 or self.current_lr_value != 90:
                            self.reset_servo()
                            self.idle_count = 0
                        else:
                            self.idle_count = 0
                else:
                    # 開始追蹤
                    success, point = self.tracker.update(frame)   # 追蹤成功後，不斷回傳左上和右下的座標
                    if success and point:
                        p1, p2 = [], []
                        print(f'point = {point}')
                        p1 = [int(point[0]), int(point[1])]
                        p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
                        print(f'p1 = {p1}')
                        print(f'p2 = {p2}')
                        # 根據座標，繪製四邊形，框住要追蹤的物件
                        cv2.rectangle(frame, tuple(p1), tuple(p2), (0, 0, 255), 3)  
        
                    # 追蹤移動攝影機對齊追蹤物體的中心
                    if  p1 and p2:
                        self.tracking_ud = (p1[1] + p2[1]) / 2
                        self.tracking_lr = (p1[0] + p2[0]) / 2
                        
                        # 計算需要位移的量
                        self.count_diff()
                        
                        # 移動鏡頭
                        self.move_updown(self.diff_ud)
                        self.move_leftright(self.diff_lr)
                    
            self.cap.release()
            cv2.destroyAllWindows()
        except:
            x = traceback.format_exc()
            print(x)

if __name__ == '__main__':
    controller = servo_control('COM4')


