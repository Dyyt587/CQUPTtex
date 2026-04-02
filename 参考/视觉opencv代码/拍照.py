import cv2

width, height = 640, 480
light_x, light_y = 319, 243
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("错误：无法打开摄像头。")
    exit()

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

i = 1
while i < 50:
    _, frame = cap.read()
    cv2.imwrite("/home/applepie/PycharmProjects/edc_opencv/pic/IR_camera_calib_img/img"+str(i)+'.jpg', frame, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
    cv2.imshow('frame', frame)
    i += 1
    if cv2.waitKey(200) & 0xFF == 27: # 按ESC键退出
        break
cv2.destroyAllWindows()