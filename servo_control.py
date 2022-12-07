import pigpio
import time
import cv2
import traceback

# 設置連線到哪
pi = pigpio.pi('192.168.8.103',8888)

# 設置servo 針腳
servo_updown = 19
servo_leftright = 26

# 將值傳送至針腳
# 安全範圍是介於 1000~2000
# 置中為 1500

def servo_move(
    current_ud,
    current_lr,
    center_ud,
    center_lr,
    track_ud,
    track_lr,
    move_rate = 1,
    ):
    diff_ud =  track_ud - center_ud 
    if  diff_ud != 0:
        #diff_ud_value = diff_ud / abs(diff_ud) * move_rate
        if diff_ud > 100:
            diff_ud_value = int(diff_ud // 10)
        else:
            diff_ud_value = int(diff_ud // 100)
    else:
        diff_ud_value = 0
    diff_lr =  center_lr - track_lr
    if diff_lr != 0:
        #diff_lr_value = diff_lr / abs(diff_lr) * move_rate
        diff_lr_value = int(diff_lr // 10)
    else:
        diff_lr_value = 0
    
    # 限制上下數值
    new_ud = current_ud + diff_ud_value
    print(f'new_ud = {new_ud}')
    if new_ud < 1000:
        print(f'>>追蹤物體超出 servo 最下範圍 !!!')
        new_ud = 1000
    elif new_ud > 2000:
        print(f'>>追蹤物體超出 servo 最上範圍 !!!')
        new_ud = 2000
    new_ud = int(new_ud)

    # 限制左右數值
    new_lr = current_lr + diff_lr_value
    print(f'new_lr = {new_lr}')
    if new_lr < 1000:
        print(f'>>追蹤物體超出 servo 最左範圍 !!!')
        new_lr = 1000
    elif new_lr > 2000:
        print(f'>>追蹤物體超出 servo 最右範圍 !!!')
        new_lr = 2000
    new_lr = int(new_lr)
    
    print(f'>>[servo移動] x = {new_lr}, y = {new_ud}')
    #input()
    return new_lr, new_ud

# 紀錄 servo 目前數值
servo_ud = 1500
servo_lr = 1500

# 將攝影機 servo 置中
pi.set_servo_pulsewidth(servo_updown, 1500)
pi.set_servo_pulsewidth(servo_leftright, 1500)

tracker = cv2.TrackerCSRT_create()  # 創建追蹤器
tracking = False                    # 設定 False 表示尚未開始追蹤

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# 取得影像的尺寸大小
video_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
video_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

center_ud =  video_height / 2
center_lr =  video_width / 2

print(f"Image Size: {video_width}, {video_height}")

p1 = None
p2 = None

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Cannot receive frame")
            break
        #frame = cv2.resize(frame,(540,300))  # 縮小尺寸，加快速度
        keyName = cv2.waitKey(1)

        if keyName == ord('q'):
            break
        if keyName == ord('a'):
            area = cv2.selectROI('oxxostudio', frame, showCrosshair=False, fromCenter=False)
            tracker.init(frame, area)    # 初始化追蹤器
            tracking = True              # 設定可以開始追蹤
        if tracking:
            success, point = tracker.update(frame)   # 追蹤成功後，不斷回傳左上和右下的座標
            if success:
                p1 = [int(point[0]), int(point[1])]
                p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
                cv2.rectangle(frame, p1, p2, (0,0,255), 3)   # 根據座標，繪製四邊形，框住要追蹤的物件

            # 追蹤移動攝影機對齊追蹤物體的中心
            if  p1 and p2:
                track_ud = (p1[1] + p2[1]) / 2
                track_lr = (p1[0] + p2[0]) / 2
                new_lr, new_ud = servo_move(
                                    current_ud = servo_ud,
                                    current_lr = servo_lr,
                                    center_ud = center_ud,
                                    center_lr = center_lr,
                                    track_ud = track_ud,
                                    track_lr = track_lr,
                                    move_rate = 1,
                                    )
               
                servo_ud = new_ud
                servo_lr = new_lr
                #print(f'>>[servo移動] x = {servo_lr}, y = {servo_ud}')
               
                pi.set_servo_pulsewidth(servo_updown, new_ud)
                pi.set_servo_pulsewidth(servo_leftright, new_lr)
                


        cv2.imshow('oxxostudio', frame)


    cap.release()
    cv2.destroyAllWindows()
except:
    x = traceback.format_exc()
    print(x)
    
# 將攝影機 servo 置中
pi.set_servo_pulsewidth(servo_updown, 1500)
pi.set_servo_pulsewidth(servo_leftright, 1500)