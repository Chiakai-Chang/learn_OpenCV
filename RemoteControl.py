# -*- coding: utf-8 -*-
"""
Created on Wed Dec  6 11:19:53 2023

@author: Chiakai
"""

import serial
import time
import cv2
import traceback
import json

def send_servo_command(ser, servo_number, position):
    """
    向 ESP32S3 發送伺服器馬達控制命令。
    :param ser:序列埠對象
    :paramservo_number: 伺服馬達編號
    :param position: 伺服馬達的目標位置
    """
    command = {"servoNumber": servo_number, "position": position}
    print(command)
    json_command = json.dumps(command) + "\n"  # 转换为 JSON 并添加换行符
    ser.write(json_command.encode('utf-8'))
    time.sleep(0.02)

class LuminaGuardian:
    
    # 追蹤器名稱
    name = 'Lumina Guardian'
    
    # 版本號
    version = '20231224_1'
    
    # Pin 
    Pin_up_down    = 1 # 黃線, PIN 10, 40 ~ 160
    Pin_left_right = 2 # 白線, PIN 13, 40 ~ 160
    Pin_on_off     = 3 # 橘線, PIN 3,  70 == on, 180 == off

    # 紀錄 servo 目前數值
    current_ud_value = 90
    current_lr_value = 90
    current_onoff = 1
    
    # Tracking Box 左上、右下座標
    frame = None
    tracking = False
    tracking_left_up = [-1, -1]
    tracking_right_down = [-1, -1]
    tracking_ud = -1 # Tracking Box 中央 y
    tracking_lr = -1 # Tracking Box 中央 x
    
    # 移動量計算方式
    easy_count = False # True 時就是每次只動 1
    
    # 若誤差不大就暫不移動
    ignore_move_diff_ratio = 0.05
    
    # 若太久就自動 Reset
    idle_start = 0
    idle_count = 0
    # 當 light off，且
    # idle_count > idle_to_reset 時 Reset
    idle_to_reset = 5 # 秒
    
    # 與鏡頭畫面互動
    mouse_xy = [] # 當前滑鼠座標位置
    
    # tracking box 預設大小
    # 因為用 ROI 框左上右下的方式，操作太複雜沒有爽感
    # 所以改用直接一點就追蹤
    tracking_box_size = 0.2
    # tracking_box = (x, y, w, h)
    # x 和 y 座標標識了選定區域的起始點 (0, 0 在左上)
    # w 和 h 描述了從這個起點開始的區域的寬度和高度
    tracking_box = [] 
    
    # 啟動追蹤條件
    track_wait_start = 0
    hold_to_activate_tracking = 0.1
    hold_lapse_time = 0
    current_tracking_area = [] # 當下 Tracking 的位置
    
    # 繪製追蹤的 Box
    tracking_point_to_draw = []
    
    # 物件消失後，多久恢復原位
    reset_count = 0
    time_to_reset = 150 # 個 loop
    
    # 模式切換
    # 0 = 預設 (移動模式: 距離變速模式)
    # 1 = 經典 (移動模式: 固定一步模式)
    mode_option = [
        '移動模式: 距離變速模式',
        '移動模式: 固定一步模式',
        ]
    mode_idx = 0
    
    # 滑鼠事件
    mouse_event = None 
    mouse_clicked_not_up = False
    
    def chang_mode(self):
        self.mode_idx += 1 
        if self.mode_idx >= len(self.mode_option):
            self.mode_idx = 0
        print(f'>>切換模式{self.mode_idx + 1}: {self.mode_option[self.mode_idx]}')
    
    def __init__(
            self, 
            # Serial 要通訊的 COM_PORT
            COM_PORT_udlr,
            COM_PORT_on,
            Baud_rate = 115200,
            # 鏡頭編號
            camera_id = 1, # 因為筆電幾乎都有內建，所以預設不是 0
            # 點擊以後，要以多大的 Box 去追蹤
            # 是相對於顯示畫面的大小比例
            tracking_box_size = 0.2,
            ):
    
        self.COM_PORT_udlr = COM_PORT_udlr
        self.COM_PORT_on = COM_PORT_on
        self.Baud_rate = Baud_rate
        self.camera_id = camera_id
        
        # Serial 溝通物件
        if COM_PORT_udlr == COM_PORT_on:
            self.controller = serial.Serial(self.COM_PORT_udlr, self.Baud_rate, timeout=1)
            self.controller_on = self.controller
        else:
            self.controller = serial.Serial(self.COM_PORT_udlr, self.Baud_rate, timeout=1)
            self.controller_on = serial.Serial(self.COM_PORT_on, self.Baud_rate, timeout=1)
        
        print('>>Servo 已啟動...')
        self.reset_servo()
        
        # CV2 的鏡頭物件
        print('>>鏡頭啟動中，請稍候...')
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
        
        # 創建追蹤器
        # 特別註記
        # 以下的 TrackerCSRT_create 必須要以下版本才能用
        # pip install opencv-contrib-python==4.5.1.48
        self.tracker = cv2.TrackerCSRT_create()  
        
        # 計算 Tracking_Box_size
        self.tracking_box_size = tracking_box_size
        self.tracking_width = self.video_width * self.tracking_box_size
        self.tracking_height = self.video_height * self.tracking_box_size

        self.run()
        print('>>已開啟鏡頭...')        
        
    def click_to_box(self, x_center, y_center):
        # 計算點擊位置為中心，延伸的 Box 參數
        # (x, y, w, h)
        # x 和 y 座標標識了選定區域的起始點(0, 0 在左上)
        # w 和 h 描述了從這個起點開始的區域的寬度和高度
        x = x_center - (self.tracking_width / 2)
        if x < 0:
            x = 0 # 以免 box 超出鏡頭
        
        y = y_center - (self.tracking_height / 2)
        if y < 0:
            y = 0 # 以免 box 超出鏡頭
            
        w = self.tracking_width
        if (x + w) > self.video_width:
            # 以免 box 超出鏡頭
            w = self.video_width - x
            
        h = self.tracking_height
        if (y + h) > self.video_height:
            # 以免 box 超出鏡頭
            w = self.video_height - y
        
        self.tracking_box = [x, y, w, h]
        return self.tracking_box

    def reset_servo(self):
        # 將攝影機 servo 置中
        self.current_ud_value = 90
        self.current_lr_value = 90
        self.current_onoff = 0
        
        send_servo_command(
            self.controller,
            self.Pin_up_down,
            self.current_ud_value,
            )
        send_servo_command(
            self.controller,
            self.Pin_left_right,
            self.current_lr_value,
            )
        send_servo_command(
            self.controller_on,
            self.Pin_on_off,
            180,
            )
        
        print('>>Servo 位置已歸零...')
    
    def light_on(self):
        self.current_onoff = 1
        command = {"servoNumber": self.Pin_on_off, "position": 70}
        json_command = json.dumps(command) + "\n" 
        print(json_command)
        self.controller_on.write(json_command.encode('utf-8'))

    def light_off(self):
        self.current_onoff = 0
        command = {"servoNumber": self.Pin_on_off, "position": 180}
        json_command = json.dumps(command) + "\n" 
        print(json_command)
        self.controller_on.write(json_command.encode('utf-8'))
    
    def count_move_rate(self, diff):
        # 計算要移動的距離，是佔畫面一半的多少比例
        # 若是 1/2 以內，都只移動 1
        # 若是 1/2 ~ 3/4 內，移動 2
        # 若是 3/4 ~ 1 內，移動 3
        if abs(diff) <= self.ignore_move_diff_ratio:
            step = 0
            return int(step)
        
        if abs(diff) <= 0.3:
            step = 1 
        elif abs(diff) <= 0.4:
            step = 2
        elif abs(diff) <= 0.6:
            step = 3
        elif abs(diff) <= 0.8:
            step = 4
        else:
            step = 5
        
        # 只取正負數
        if diff < 0:
            step = step * -1
        elif diff == 0:
            step = 0
        return int(step)
    
    def count_diff(
            self,
            x = None,
            y = None,
            ):
        if x and y:
            self.diff_ud =  y - self.center_ud
            self.diff_lr =  self.center_lr - x
        else:
            self.diff_ud =  self.tracking_ud - self.center_ud
            self.diff_lr =  self.center_lr - self.tracking_lr
        #print(f'center_ud = {self.center_ud}')
        #print(f'center_lr = {self.center_lr}')
        #print(f'diff_ud = {self.diff_ud}')
        #print(f'diff_lr = {self.diff_lr}')
        
        # return 該 diff 是畫面長寬一半的比例
        self.moving_rate_ud = self.diff_ud / (self.video_height/2)
        self.step_ud = self.count_move_rate(self.moving_rate_ud)
        print(f'>>因 y 軸距離為: {self.moving_rate_ud}')
        print(f'>>所以 y 軸決定移動: {self.step_ud}')
        
        self.moving_rate_lr = self.diff_lr / (self.video_width/2)
        self.step_lr = self.count_move_rate(self.moving_rate_lr)
        print(f'>>因 x 軸距離為: {self.moving_rate_lr}')
        print(f'>>所以 x 軸決定移動: {self.step_lr}')
    
    def move_updown(self, value):
        if value == 0:
            return None
        new_ud_value = self.current_ud_value + value
        # 必須要確保是整數
        new_ud_value = int(new_ud_value)
        if 30 <= new_ud_value <= 160:
            self.current_ud_value = new_ud_value
            send_servo_command(
                self.controller, 
                self.Pin_up_down,
                self.current_ud_value,
                )
        else:
            print(f'[Limit] Servo ud 想移動到: {new_ud_value}, 但已超出可移動範圍...')
        self.show_servo_position()

    def move_leftright(self, value):
        if value == 0:
            return None
        new_lr_value = self.current_lr_value + value
        # 必須要確保是整數
        new_lr_value = int(new_lr_value)
        if 30 <= new_lr_value <= 160:
            self.current_lr_value = new_lr_value
            send_servo_command(
                self.controller, 
                self.Pin_left_right,
                self.current_lr_value,
                )
        else:
            print(f'[Limit] Servo lr 想移動到: {new_lr_value}, 已超出可移動範圍...')
        self.show_servo_position()

    def mouse_action(self, event, x, y, flags, frame):
        self.frame = frame
        self.mouse_xy = [x, y]
        self.mouse_event = [event, x, y, flags, frame]
        
        if event == cv2.EVENT_LBUTTONDOWN:
           print(f'>>Event: {event}')
           print(f'Position X: {x}')
           print(f'Position Y: {y}')
           
           # 計算大約按半秒以上才會追蹤
           self.track_wait_start = time.time()
           self.count_diff(x, y)
           
           self.mouse_clicked_not_up = True
           
           # 如果開啟著 Tracking 點 Box 就關閉 Tracking
           if self.tracking:
               tx, ty, tw, th = self.current_tracking_area
               if tx <= x <= (tx+tw):
                   if ty <= y <= (ty + th):
                       print('>>停止追蹤...')
                       self.tracking = False
                       #self.light_off()
                       self.current_tracking_area = []
        
        if event == cv2.EVENT_RBUTTONDOWN:
            # 右鍵開關燈
            print(f'>>Event: {event}')
            print(f'Position X: {x}')
            print(f'Position Y: {y}')
            
            if self.current_onoff:
                self.light_off()
            else:
                self.light_on()
        
        if event == cv2.EVENT_LBUTTONUP:
            self.mouse_clicked_not_up = False
            
            # 計算是否是在 Tracking Box
            self.hold_lapse_time = time.time() - self.track_wait_start
            if self.hold_lapse_time >= self.hold_to_activate_tracking:
                # 計算 Tracking box
                area = self.click_to_box(x, y)
                print(f'>>開始追蹤! (x:{x}, y:{y}')
                print(f'Tracking Box: {self.tracking_box}')
                self.tracker.init(frame, area)    # 初始化追蹤器
                self.tracking = True              # 設定可以開始追蹤
                self.reset_count = 0
                #self.light_on()
    
    def show_servo_position(self):
        print('>>當前 Servo Postions:')
        print(f'[UD] {self.current_ud_value}')
        print(f'[LR] {self.current_lr_value}')
        print(f'[OF] {self.current_onoff}')        
    
    def run(self):
        p1, p2 = [], []
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Cannot receive frame")
                    break
                
                cv2.setMouseCallback(self.name, self.mouse_action, frame)
    
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
                    p1, p2 = [], []
                    if success and point:
                        print('>>正在追蹤:')
                        print(f'point = {point}')
                        p1 = [int(point[0]), int(point[1])]
                        p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
                        print(f'p1 = {p1}')
                        print(f'p2 = {p2}')
                        self.current_tracking_area = point
                    else:
                        self.reset_count += 1
                        print(f'>>追蹤的物件消失，({self.reset_count}/{self.time_to_reset}輪)後重置，回復原位')
                        if self.reset_count >= self.time_to_reset:
                            self.tracking = False
                            self.reset_servo()
                            self.reset_count = 0
                    
                    # 追蹤移動攝影機對齊追蹤物體的中心
                    if  p1 and p2:
                        # 根據座標，繪製四邊形，框住要追蹤的物件
                        cv2.rectangle(frame, tuple(p1), tuple(p2), (0, 0, 255), 3)
                        
                        self.tracking_ud = (p1[1] + p2[1]) / 2
                        self.tracking_lr = (p1[0] + p2[0]) / 2
                        
                        # 計算需要位移的量
                        self.count_diff(self.tracking_lr, self.tracking_ud)
                        
                        # 移動鏡頭
                        self.move_updown(self.step_ud)
                        self.move_leftright(self.step_lr)
                   
                if self.mouse_clicked_not_up:
                    # 點擊不放，就持續往該點位方向移動
                    self.count_diff(self.mouse_xy[0], self.mouse_xy[1])
                    
                    # 移動鏡頭
                    if self.easy_count:
                        if self.diff_ud < 0:
                            self.move_updown(1)
                        elif self.diff_ud > 0:
                            self.move_updown(-1)
                        
                        if self.diff_lr < 0:
                            self.move_leftright(1)
                        elif self.diff_lr > 0:
                            self.move_leftright(-1)
                    else:
                        # 用移動量去計算
                        self.move_updown(self.step_ud)
                        self.move_leftright(self.step_lr)
            
                # 繪製所有說明文字
                try:
                    cv2.putText(frame, f'X={self.mouse_xy[0]}, Y={self.mouse_xy[1]}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                except:
                    pass
                #frame = cv2.resize(frame,(540,300))  # 縮小尺寸，加快速度
                # 將按鈕說明寫在圖上
                cv2.putText(frame, self.name, (int(self.video_width - 180), int(self.video_height-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                cv2.putText(frame, 'Mouse:', (10, int(self.video_height-170)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                cv2.putText(frame, 'Hold to Track, Click to Cancel', (10, int(self.video_height-140)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                cv2.putText(frame, 'Buttons:', (10, int(self.video_height-110)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                cv2.putText(frame, '"m": Change Move Mode', (10, int(self.video_height-80)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                cv2.putText(frame, '"r": Reset Camera', (10, int(self.video_height-50)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)
                cv2.putText(frame, '"q": quit...', (10, int(self.video_height-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2)

                cv2.imshow(self.name, frame)
                keyName = cv2.waitKey(1)  & 0xFF
                
                if keyName == ord('q'):
                    break
                if keyName == ord('r'):
                    self.tracking = False
                    self.reset_servo()
                if keyName == ord('s'):
                    self.show_servo_position()
                if keyName == ord('m'):
                    self.chang_mode()
                if keyName == ord('a'):
                    area = cv2.selectROI(self.name, frame, showCrosshair=False, fromCenter=False)
                    print(f'area = {area}')
                    self.tracker.init(frame, area)    # 初始化追蹤器
                    self.tracking = True              # 設定可以開始追蹤
                    #self.light_on()
                    
            self.cap.release()
            cv2.destroyAllWindows()
        except:
            x = traceback.format_exc()
            print(x)

if __name__ == '__main__':
    Guardian = LuminaGuardian(
        # 控制上下左右的 Serial Port
        'COM7', 
        # 控制開關的 Serial Port (可以同上)
        'COM4',
        # 點擊以後自動繪製的 tracking_box 佔畫面比例
        tracking_box_size = 0.2,
        )


