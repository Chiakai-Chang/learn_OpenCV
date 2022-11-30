# Import packages
import cv2


# 設定滑鼠點擊到範圍要顯示與否的開關
track_BBoxs = dict()
box_len = 100

# 計算是否是已記載的 BBox
def find_Match_BBox(x, y, track_BBoxs):
    for bid, data in track_BBoxs.items():
        bx, by, bx1, by1 = data['range']
        if (bx <= x <= bx1) and (by <= y <= by1):
            return bid

# 可以輪用的顏色表
bbox_color = [
    (255, 0, 0),
    (255, 165, 0),
    (255, 255, 0),
    (0, 255, 0),
    (0, 0, 255),
    (106, 90, 205),
    (160, 32, 240),
    (220, 220, 220),
    ]

# 繪畫 BBox
def draw_bbox(img):
    global track_BBoxs
    global bbox_color
    
    color_len = len(bbox_color)
    
    for bid, data in track_BBoxs.items():
        bx, by, bx1, by1 = data['range']
        
        onoff = data['onoff']
        if onoff:
            color_index = bid % color_len
            cv2.rectangle(img, (bx, by), (bx1, by1), bbox_color[color_index], 2)
    return img
 
# read from camera
cap = cv2.VideoCapture(0) 

# 點擊後動作判斷
def mouse_action(event, x, y, flags, frame):
    global track_BBoxs
    global box_len
    
    
    color_len = len(bbox_color)
    
    if event == cv2.EVENT_LBUTTONDOWN:
        bid = find_Match_BBox(x, y, track_BBoxs)
        if bid:
            # 存在就開關
            track_BBoxs[bid]['onoff'] = not track_BBoxs[bid]['onoff']
        else:
            # 不存在就新增
            try:
                max_bid = max(list(track_BBoxs.keys())) + 1
            except:
                max_bid = 1
            track_BBoxs[max_bid] = {
            'range' : (x-box_len, y-box_len, x+box_len, y+box_len),
            'onoff' : True,
            }
    print(f'track_BBoxs = {track_BBoxs}')

# window
cv2.namedWindow('Cam')

# Close the window when key q is pressed
k = 0
while k!= ord('q'):
  ret, frame = cap.read()
  cv2.setMouseCallback('Cam', mouse_action, frame)
  # Display the image
  cv2.imshow("Cam", draw_bbox(frame))
  k = cv2.waitKey(1)  & 0xFF
  # If c is pressed, clear the window, using the dummy image
  if k == ord('c'):
    image= temp.copy()
    cv2.imshow("Cam", image)

print(f'records = {records}') 

cap.release() 
cv2.destroyAllWindows()