import cv2
import numpy as np
import math
import serial
import time
import threading

def task(non):
    pass

port = '/dev/ttyAMA2'
baudrate = 115200
try:
    ser = serial.Serial(port=port,
                        baudrate=baudrate)
except serial.SerialException as e:
    print(f"无法打开串口{port}: {e}")

print(f"串口{port}已打开，波特率{baudrate}")

ser.write("begin".encode("utf-8"))

width = 640
height = 480

color_lower = [0,0,0]
color_upper = [160,160,160]#180,255,80HSV  160,160,160BGR

#摄像头参数设置
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

cv2.namedWindow('Trackbar',256)
cv2.createTrackbar('H_Min','Trackbar',color_lower[0],255,task)
cv2.createTrackbar('S_Min','Trackbar',color_lower[1],255,task)
cv2.createTrackbar('V_Min','Trackbar',color_lower[2],255,task)

cv2.createTrackbar('H_Max','Trackbar',color_upper[0],255,task)
cv2.createTrackbar('S_Max','Trackbar',color_upper[1],255,task)
cv2.createTrackbar('V_Max','Trackbar',color_upper[2],255,task)

def calculate_centroid(contour):
    """计算轮廓的质心"""
    M = cv2.moments(contour)
    # 防止除以零
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)
def merge_close_contours(contours, area_diff_thresh=0.1, centroid_dist_thresh=10):
    """
    根据面积和质心距离合并轮廓。

    Args:
        contours (list): 初始轮廓列表。
        area_diff_thresh (float): 面积差异百分比阈值。例如 0.1 表示 10%。
        centroid_dist_thresh (int): 质心距离阈值（像素）。

    Returns:
        list: 合并后的轮廓列表。
    """
    if not contours:
        return []

    # 创建一个布尔列表来标记哪些轮廓需要保留，初始时全部保留
    to_keep = [True] * len(contours)
    
    # 预先计算所有轮廓的面积和质心，避免重复计算
    contour_properties = []
    for contour in contours:
        area = cv2.contourArea(contour)
        centroid = calculate_centroid(contour)
        contour_properties.append({'area': area, 'centroid': centroid, 'contour': contour})

    # 按面积从大到小排序，这有助于优先保留外轮廓
    contour_properties.sort(key=lambda p: p['area'], reverse=True)
    
    # 更新 contours 和 to_keep 列表以匹配排序后的顺序
    contours = [p['contour'] for p in contour_properties]
    to_keep = [True] * len(contours)

    # 遍历每一对轮廓进行比较
    for i in range(len(contours)):
        # 如果当前轮廓已经被标记为删除，则跳过
        if not to_keep[i]:
            continue

        prop1 = contour_properties[i]
        # 如果质心计算失败，则跳过
        if prop1['centroid'] is None:
            continue

        # 与后面的所有轮廓进行比较
        for j in range(i + 1, len(contours)):
            if not to_keep[j]:
                continue
            
            prop2 = contour_properties[j]
            if prop2['centroid'] is None:
                continue

            # 1. 比较面积
            # 避免除以零
            max_area = max(prop1['area'], prop2['area'])
            if max_area == 0:
                continue
            
            area_diff_ratio = abs(prop1['area'] - prop2['area']) / max_area
            
            # 2. 比较质心
            cx1, cy1 = prop1['centroid']
            cx2, cy2 = prop2['centroid']
            centroid_dist = math.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2)

            # 3. 判断是否满足合并条件
            if area_diff_ratio < area_diff_thresh and centroid_dist < centroid_dist_thresh:
                # 满足条件，标记面积较小的轮廓为删除 (因为我们已经排序，j总是指向较小的)
                to_keep[j] = False
                # print(f"合并轮廓 {i} 和 {j}，因为面积差异: {area_diff_ratio:.2f}, 质心距离: {centroid_dist:.2f}")

    # 收集所有需要保留的轮廓
    final_contours = [contours[i] for i in range(len(contours)) if to_keep[i]]
    
    return final_contours
def order_points(pts):
    """
    对矩形的四个角点进行排序：左上、右上、右下、左下。
    这对于cv2.getPerspectiveTransform至关重要。
    """
    # 确保输入是 float32 类型的 NumPy 数组
    pts = np.array(pts, dtype="float32")

    # 关键步骤：如果输入形状是 (N, 1, 2)，则将其重塑为 (N, 2)
    # 这使得后续的 sum 和 diff 操作可以直接作用于 x, y 坐标
    if pts.ndim == 3 and pts.shape[1] == 1:
        pts = pts.reshape(pts.shape[0], 2)
        # 验证点数，确保是 4 个点
    if len(pts) != 4:
        raise ValueError(f"输入点数不是 4 个，实际为 {len(pts)} 个。无法排序矩形角点。")

        # 初始化一个存储排序后点的数组
    rect = np.zeros((4, 2), dtype="float32")

    # 1. 找到左上角点和右下角点
    # 左上角的点 (x + y 和最小)
    # 右下角的点 (x + y 和最大)
    s = pts.sum(axis=1)  # 计算每个点的 x + y
    rect[0] = pts[np.argmin(s)]  # 左上
    rect[2] = pts[np.argmax(s)]  # 右下

    # 2. 找到右上角点和左下角点
    # 右上角的点 (x - y 差最小)
    # 左下角的点 (x - y 差最大)
    diff = np.diff(pts, axis=1).flatten() # 计算每个点的 x - y，并展平为一维数组
    rect[1] = pts[np.argmin(diff)] # 右上
    rect[3] = pts[np.argmax(diff)] # 左下
    return rect
def perspective_transform_a4(image, distorted_rect_coords):
    """
    将畸变矩形进行透视变换为正常A4纸比例的矩形，
    找到其中点，并对应到原图的坐标点。

    Args:
        image (np.array): 原始图像。
        distorted_rect_coords (list or np.array): 畸变矩形的四个角点坐标，
                                                 例如 [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]。
                                                 点的顺序不重要，函数内部会进行排序。

    Returns:
        tuple: (warped_image, original_center_point_coords)
               - warped_image (np.array): 经过透视变换后的图像（标准A4矩形）。
               - original_center_point_coords (tuple): 矫正后矩形中心点在原图上的坐标 (x, y)。
                                                     如果无法进行透视变换，则返回 None。
    """

    # 确保输入坐标是 NumPy 数组
    pts = np.array(distorted_rect_coords, dtype="float32")
    ordered_pts = order_points(pts)
    # # 对角点进行排序
    # ordered_pts = order_points(pts)
    (tl, tr, br, bl) = ordered_pts
    # --- 常量定义 ---
    # A4 纸的物理尺寸 (毫米)
    A4_WIDTH_MM = 297
    A4_HEIGHT_MM = 210
    # 圆的半径 (厘米)
    CIRCLE_RADIUS_CM = 6
    CIRCLE_RADIUS_MM = CIRCLE_RADIUS_CM * 10
    circle_radius_mm = CIRCLE_RADIUS_MM
    # 定义 A4 纸的宽高比
    # 假设A4纸的宽度为 210，高度为 297，或者反过来，我们取一个常用的尺寸作为基准
    # 比如 800像素宽的A4纸
    A4_WIDTH = 800
    A4_HEIGHT = int(A4_WIDTH * (210 / 297))  # A4比例 297/210 约等于 1.414
    PIXEL_TO_MM_RATIO = A4_WIDTH_MM / A4_WIDTH
    MM_TO_PIXEL_RATIO = A4_WIDTH / A4_WIDTH_MM  # 毫米到像素的比例

    # 定义目标矩形的四个角点（标准 A4 比例）
    dst = np.array([
        [0, 0],  # 左上
        [A4_WIDTH - 1, 0],  # 右上
        [A4_WIDTH - 1, A4_HEIGHT - 1],  # 右下
        [0, A4_HEIGHT - 1]  # 左下
    ], dtype="float32")

    # 计算透视变换矩阵 (M)
    M = cv2.getPerspectiveTransform(ordered_pts, dst)
    # 计算逆透视变换矩阵 (Minv)
    Minv = cv2.getPerspectiveTransform(dst, ordered_pts)
    # 执行透视变换
    warped = cv2.warpPerspective(image, M, (A4_WIDTH, A4_HEIGHT))

    # --- 1. 计算 A4 纸的物理中心点 ---
    a4_center_x_mm = A4_WIDTH_MM / 2
    a4_center_y_mm = A4_HEIGHT_MM / 2
    # 将 A4 纸的物理中心点转换回标准 A4 纸像素坐标
    a4_center_pixel_on_a4 = np.array([[
        a4_center_x_mm * MM_TO_PIXEL_RATIO,
        a4_center_y_mm * MM_TO_PIXEL_RATIO
    ]], dtype="float32")
    # 将 A4 纸的中心点从标准 A4 纸像素坐标映射回原始图像像素坐标
    a4_center_on_original_image = cv2.perspectiveTransform(a4_center_pixel_on_a4.reshape(-1, 1, 2), Minv)[0][0]
    a4_center_original_x = int(a4_center_on_original_image[0])
    a4_center_original_y = int(a4_center_on_original_image[1])

    # 找到矫正后矩形的中心点（在 warped 图像中）
    warped_center_x = A4_WIDTH / 2
    warped_center_y = A4_HEIGHT / 2
    warped_center_point = np.array([[[warped_center_x, warped_center_y]]], dtype="float32")

    # 获取摄像头图像的中心像素点
    camera_center_x_pixel = image.shape[1] / 2
    camera_center_y_pixel = image.shape[0] / 2
    camera_center_pixel_coords = np.array([[[camera_center_x_pixel, camera_center_y_pixel]]], dtype="float32")

    # 将矫正后的中心点映射回原始图像
    original_center_point = cv2.perspectiveTransform(warped_center_point, Minv)
    camera_center_on_a4_pixels = cv2.perspectiveTransform(camera_center_pixel_coords, M)[0][0]
    # 将摄像头中心点从 A4 纸像素坐标转换为 A4 纸物理坐标 (毫米)
    camera_center_on_a4_mm_x = camera_center_on_a4_pixels[0] * PIXEL_TO_MM_RATIO
    camera_center_on_a4_mm_y = camera_center_on_a4_pixels[1] * PIXEL_TO_MM_RATIO
    # --- 4. 计算摄像头中心与 A4 纸中心的物理距离差值 ---
    delta_x_mm = camera_center_on_a4_mm_x - a4_center_x_mm
    delta_y_mm = camera_center_on_a4_mm_y - a4_center_y_mm  # 注意这里的Y轴，如果A4纸坐标系Y轴向下，则y_cam - y_a4_center

    # 提取映射回原图的中心点坐标
    original_center_x = original_center_point[0][0][0]
    original_center_y = original_center_point[0][0][1]

    # --- 5. 绘制圆 (以 A4 纸物理中心为圆心) ---
    # 检查圆是否完全在 A4 纸的物理范围内 (以 A4 纸中心为基准)
    # 圆心： (A4_WIDTH_MM/2, A4_HEIGHT_MM/2)
    # 边界检查需要考虑圆心和半径
    if (a4_center_x_mm - circle_radius_mm < 0 or
            a4_center_x_mm + circle_radius_mm > A4_WIDTH_MM or
            a4_center_y_mm - circle_radius_mm < 0 or
            a4_center_y_mm + circle_radius_mm > A4_HEIGHT_MM):
        cv2.putText(image, "Circle Out of A4 Bounds!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return image, delta_x_mm, delta_y_mm  # 即使超出范围，仍然返回差值

    # 生成圆上的点 (在 A4 纸的物理坐标系中，以A4纸中心为圆心)
    circle_points_mm = []
    num_points = 100  # 圆的离散点数量，越多越平滑
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        # 圆心是 A4 纸的物理中心
        x_mm = a4_center_x_mm + circle_radius_mm * math.cos(angle)
        y_mm = a4_center_y_mm + circle_radius_mm * math.sin(angle)
        circle_points_mm.append([x_mm, y_mm])

    # 将 A4 纸物理坐标点转换回标准 A4 纸像素坐标
    circle_points_pixels_on_a4 = np.array(
        [[p[0] * MM_TO_PIXEL_RATIO, p[1] * MM_TO_PIXEL_RATIO] for p in circle_points_mm],
        dtype="float32"
    )

    # 将圆点从标准 A4 纸像素坐标映射回原始图像像素坐标
    circle_points_original_image = cv2.perspectiveTransform(
        circle_points_pixels_on_a4.reshape(-1, 1, 2), Minv
    ).reshape(-1, 2)

    # 将浮点坐标转换为整数，以便绘制
    circle_points_original_image_int = np.int32(circle_points_original_image)

    # 在原图上绘制圆
    cv2.polylines(image, [circle_points_original_image_int], True, (0, 0, 255), 1)  # 红色圆

    # 显示信息
    cv2.putText(image, f"A4 Center: ({a4_center_original_x}, {a4_center_original_y})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(image, f"Cam Center: ({int(camera_center_x_pixel)}, {int(camera_center_y_pixel)})",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(image, f"Delta X (mm): {delta_x_mm:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(image, f"Delta Y (mm): {delta_y_mm:.1f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(image, f"Circle Radius: {CIRCLE_RADIUS_CM}cm", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 255), 2)

    return image, (original_center_x, original_center_y)

# ret, frame = cap.read()

# Lock = threading.Lock()
# def cap_read():
#     global ret
#     global frame
#     global cap
#     while cap.isOpened():
#         # ret, frame = cap.read()
#         # cv2.imshow("frame", frame)
#         time.sleep(0.01)


# def img_process():
#     global ret
#     global frame
#     global cap
#     global ser
cx = 0
cy = 0
tcx = 0
tcy = 0
while cap.isOpened():
    ret, frame = cap.read()
    if ret == True:
        # cv2.imshow("frame",frame)
        img = cv2.resize(frame, (int(width), int(height)), interpolation=cv2.INTER_LINEAR)
        img_blur = cv2.GaussianBlur(img, (3, 3), 0)
        # # cv2.imshow("img_blur",img_blur)
        # img_hsv = cv2.cvtColor(img_blur,cv2.COLOR_BGR2HSV)
        # # cv2.imshow("img_hsv",img_hsv)
        # erode_hsv = cv2.erode(img_hsv,None,iterations=1)
        # # cv2.imshow("erode_hsv",erode_hsv)

        color_lower[0] = cv2.getTrackbarPos('H_Min', 'Trackbar')
        color_lower[1] = cv2.getTrackbarPos('S_Min', 'Trackbar')
        color_lower[2] = cv2.getTrackbarPos('V_Min', 'Trackbar')

        color_upper[0] = cv2.getTrackbarPos('H_Max', 'Trackbar')
        color_upper[1] = cv2.getTrackbarPos('S_Max', 'Trackbar')
        color_upper[2] = cv2.getTrackbarPos('V_Max', 'Trackbar')

        inRange_hsv = cv2.inRange(img_blur, np.array(color_lower), np.array(color_upper))
        cv2.imshow("inRange_hsv", inRange_hsv)

        # img_bin = cv2.inRange(img_blur,cv2.scalar(0,0,0),cv2.Scalar(180,255,46));
        # cv2.imshow("img_bin",img_bin)
        # img_black = cv2.
        edges = cv2.Canny(inRange_hsv, 100, 400)
        cv2.imshow("edges", edges)

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = merge_close_contours(contours)
        send_flag = 0

        for contour in contours:
            peri = cv2.arcLength(contour, True)

            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

            if len(approx) == 4:
                area = cv2.contourArea(contour)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int_(box)
                rect_area = rect[1][0] * rect[1][1]

                if 3000 < area and area / rect_area > 0.8:
                    approx_after = [0, 0, 0, 0]
                    # 排好序的角点输出，0号是左上角，顺时针输出
                    approx_after[0] = approx[1]
                    approx_after[1] = approx[0]
                    approx_after[2] = approx[3]
                    approx_after[3] = approx[2]

                    result_img, (tcx, tcy) = perspective_transform_a4(img, approx_after)
                    # print(tcx, tcy)
                    # result_img = Perspective_transform(approx_after,img)

                    # cv2.imshow('result_img', result_img)
                    x, y, w, h = cv2.boundingRect(contour)

                    aspect_ratio = float(w) / h  # 宽高比

                    # if (float(297) / 210) - 0.5 < aspect_ratio < (float(297) / 210) + 0.5 :
                    send_flag = 1
                    cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), thickness=2, lineType=8, shift=0)
                    cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
                    cx = x + float(w) / 2
                    cy = y + float(h) / 2
                    # for i in range(0,3):
                    msg = '@' + "%.3f" % tcx + ',' + "%.3f" % tcy + ',' + '1.0,#'
                    print(tcx, tcy, 1.0)
                    ser.write(msg.encode('utf-8'))
                    # time.sleep(0.01)
        if send_flag == 1:
            send_flag = 0
        else:
            msg = '@' + "%.3f" % tcx + ',' + "%.3f" % tcy + ',' + '0.0,#'
            print(tcx, tcy, 0.0)
            ser.write(msg.encode('utf-8'))

        cv2.line(img, (int(width / 2 - 10), int(height / 2)), (int(width / 2 + 10), int(height / 2)), thickness=1,
                 color=(255, 255, 255), lineType=8, shift=0)
        cv2.line(img, (int(width / 2), int(height / 2 + 10)), (int(width / 2), int(height / 2 - 10)), thickness=1,
                 color=(255, 255, 255), lineType=8, shift=0)
        cv2.imshow('Detected Rectangles', img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
