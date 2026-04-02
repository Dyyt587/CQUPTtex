import cv2
import numpy as np
import math
import serial
import time
import threading
import queue
import re
# --- 1. 线程共享资源和控制信号 ---
# --- 摄像头内参和畸变系数 ---
# ***重要：这些是示例值，你必须通过摄像头标定来获取自己的值！***
# 摄像头内参矩阵
# fx, fy: 焦距 (像素)
# cx, cy: 主点/光学中心 (像素)
CAMERA_MATRIX = np.array([
    [435.37360222, 0, 318.68329424],
    [0, 433.86015507, 226.72463621],
    [0, 0, 1]
], dtype="float32")
#A4纸物理尺寸
A4_WIDTH_MM = 297 - 15
A4_HEIGHT_MM = 210 - 15

# 畸变系数
# k1, k2, p1, p2, k3
# DIST_COEFFS = np.array([[-0.50462529, 0.36744964, -0.00220405, 0.0050921, -0.18511531]], dtype="float32") # 假设没有畸变，实际请使用标定结果
DIST_COEFFS = np.zeros((4,1), dtype="float32")
#读文件中的阈值
def read_thresholds():
    with open('/home/applepie/PycharmProjects/edc_opencv/trackbar_values.txt','r') as f:
        lines=f.readlines()
        # print(lines)
    line0 = re.split(',|\n',lines[0])
    line1 = re.split(',|\n',lines[1])
    trackbar_values['lower'] = [int(line0[0]),int(line0[1]),int(line0[2])]
    trackbar_values['upper'] = [int(line1[0]), int(line1[1]), int(line1[2])]

#将阈值写入文件中
def save_thresholds():
    with open('/home/applepie/PycharmProjects/edc_opencv/trackbar_values.txt','r') as f:
        lines=f.readlines()
    with open('/home/applepie/PycharmProjects/edc_opencv/trackbar_values.txt','w') as f:
        temp_thresholds_lower = [str(x) for x in trackbar_values['lower']]
        temp_thresholds_upper = [str(x) for x in trackbar_values['upper']]
        lines[0] = ','.join(temp_thresholds_lower)+'\n'
        lines[1] = ','.join(temp_thresholds_upper) + '\n'
        f.writelines(lines)

# 用于在读取线程和处理线程之间传递帧的队列
frame_queue = queue.Queue(maxsize=1) 

# 用于在处理线程和发送线程之间传递数据的队列
data_queue = queue.Queue()

# 用于控制所有子线程运行的事件
running_event = threading.Event()
running_event.set() # 初始状态设置为运行

# 用于保护共享的显示图像的锁
display_lock = threading.Lock()
display_frame = None # 共享变量，用于存储最新的处理结果图像
display_inRange_mask = None
display_inRange_edges = None
# 共享字典，用于从主线程传递Trackbar值到处理线程
trackbar_values = {
    'lower': [0, 0, 0],
    'upper': [180, 255, 130]#160, 160, 160BGR
}

read_thresholds()

# --- 2. 线程函数定义 ---

def read_thread_func(cap):
    """线程1：读取视频数据"""
    print("读取线程已启动")
    while running_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("无法从摄像头读取帧，读取线程退出。")
            running_event.clear()
            break
        try:
            # 清空队列，确保只保留最新的一帧
            while not frame_queue.empty():
                frame_queue.get_nowait()
            frame_queue.put_nowait(frame)
        except queue.Full:
            pass # 如果在清空和放入的瞬间，其他线程操作了队列，就忽略这个错误
    print("读取线程已停止")


def process_thread_func():
    """线程2：处理数据 (逻辑与原始代码完全一致)"""
    global display_frame
    global display_inRange_mask
    global display_inRange_edges
    print("处理线程已启动")
    
    # 临时变量，用于在没有检测到矩形时发送上一次的坐标
    last_known_tcx = 800.0
    last_known_tcy = 240.0
    last_known_pan_deg = 0.0
    last_known_tilt_deg = 0.0
    se = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    lose_target_count = 0
    lose_target_flag = 1
    find_target_count = 0
    find_target_flag = 0
    while running_event.is_set():
        try:
            original_frame = frame_queue.get(timeout=1)
        except queue.Empty:
            continue

        # --- 以下是与您原始代码完全相同的处理逻辑 ---
        img = cv2.resize(original_frame, (width, height), interpolation=cv2.INTER_LINEAR)
        processed_img_for_display = img.copy() # 创建一个副本用于绘制和显示
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)

        img_hsv = cv2.cvtColor(img_blur,cv2.COLOR_BGR2HSV)
        erode_hsv = cv2.erode(img_hsv, None, iterations=1)

        lower_bgr = np.array(trackbar_values['lower'])
        upper_bgr = np.array(trackbar_values['upper'])

        inRange_mask = cv2.inRange(erode_hsv, lower_bgr, upper_bgr)
        inRange_mask = cv2.morphologyEx(inRange_mask,cv2.MORPH_OPEN,se)
        with display_lock:
            display_inRange_mask = inRange_mask

        # edges = cv2.Canny(inRange_mask, 100, 400)
        # with display_lock:
        #     display_inRange_edges = edges
        contours, _ = cv2.findContours(inRange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = merge_close_contours(contours)
        
        detected_in_frame = False
        
        # 这里的 tc_x 和 tc_y 是临时变量，用于接收函数返回值
        # 它们的含义与原始代码中的 tcx, tcy 完全相同
        tc_x, tc_y = 0, 0
        pan_deg, tilt_deg = 0, 0
        # last_known_tcx, last_known_tcy, last_known_pan_deg, last_known_tilt_deg = 0,0,0,0
        # find_a4_paper_contours()
        # new_contours = find_a4_paper_contours(img,contours,size_tolerance_mm=400)
        for contour in contours:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

            if len(approx) == 4:
                #轮廓面积
                area = cv2.contourArea(contour)
                # print(area)
                if 2500 < area < 100000:
                    # 轮廓外接矩形
                    x, y, w, h = cv2.boundingRect(contour)
                    print(w/h)
                    if 0.85 < w / h < 2:
                        # 轮廓旋转矩形
                        rect = cv2.minAreaRect(contour)
                        rect_area = rect[1][0] * rect[1][1]
                        # if (area < 20000 and area / rect_area > 0.9 ) or (50000 > area > 20000 and area / rect_area > 0.8 ) or (area > 50000 and area / rect_area > 0.7):
                        if area / rect_area > 0.75:
                            # 调用与原始代码行为一致的函数
                            # 它会返回绘制好的图像和像素坐标的中心点
                            if find_a4_paper_contours(approx,size_tolerance_mm=150):#find_a4_paper_contours(approx,size_tolerance_mm=50)
                                # processed_img_for_display, (tc_x, tc_y) = perspective_transform_a4(processed_img_for_display, approx)
                                processed_img_for_display, pan_deg, tilt_deg, distance_mm, tc_x, tc_y = perspective_transform_a4_pnp(processed_img_for_display, approx)
                                tc_y = tc_y - 10 + rect[1][0] * 0.01
                                tc_x = tc_x + 10 + rect[1][0] * 0.01
                                # 更新最后已知的有效坐标
                                last_known_tcx, last_known_tcy,last_known_pan_deg, last_known_tilt_deg = tc_x, tc_y, pan_deg, tilt_deg

                                detected_in_frame = True
                                # 在图像上绘制轮廓（与原始代码一致）
                                box = np.int_(cv2.boxPoints(rect))
                                # last_approx = approx.copy()
                                # last_box = box.copy()
                                cv2.drawContours(processed_img_for_display, [approx], -1, (0, 255, 0), 2)
                                cv2.drawContours(processed_img_for_display, [box], 0, (0, 0, 255), 2)
                                # print(area)
                                break # 找到一个满足条件的矩形就跳出循环

        # 根据是否检测到来准备要发送的数据
        if detected_in_frame:
            find_target_count += 1
            if find_target_count  and find_target_flag == 0 :
                find_target_count =0
                find_target_flag = 1

            if find_target_flag == 1:
                lose_target_count = 0
                lose_target_flag = 0
                data_to_send = {'x': tc_x, 'y': tc_y, 'pan_deg': pan_deg, 'tilt_deg': tilt_deg, 'flag': 1.0}
            else:

                data_to_send = {'x': 640, 'y': 240, 'pan_deg': pan_deg, 'tilt_deg': tilt_deg, 'flag': 1.0}
        else:
            lose_target_count += 1
            # 如果没检测到，发送上一次的有效坐标和1标志位
            if lose_target_count > 15 and lose_target_flag == 0:
                lose_target_count = 0
                lose_target_flag = 1
            if lose_target_flag == 1:
                find_target_flag = 0
                find_target_count = 0
                # data_to_send = {'x': last_known_tcx, 'y': last_known_tcy, 'pan_deg': pan_deg, 'tilt_deg': tilt_deg, 'flag': 1.0}
                data_to_send = {'x':640, 'y': 240, 'pan_deg': last_known_pan_deg,
                                'tilt_deg': last_known_tilt_deg, 'flag': 1.0}
            else:
                data_to_send = {'x': last_known_tcx, 'y': last_known_tcy, 'pan_deg': last_known_pan_deg,
                                'tilt_deg': last_known_tilt_deg, 'flag': 1.0}
        data_queue.put(data_to_send)
        
        # 绘制中心十字线
        cv2.line(processed_img_for_display, (int(light_x - 10), int(light_y)), (int(light_x + 10), int(light_y)), (255, 255, 255), 1)
        cv2.line(processed_img_for_display, (int(light_x), int(light_y + 10)), (int(light_x), int(light_y - 10)), (255, 255, 255), 1)
        
        # 使用锁更新全局显示帧
        with display_lock:
            display_frame = processed_img_for_display

    print("处理线程已停止")


def send_thread_func(ser):
    """线程3：发送数据"""
    print("发送线程已启动")
    while running_event.is_set():
        try:
            data = data_queue.get(timeout=1)
            
            # 格式化消息，与原始代码完全一致
            msg = f"@{data['x']:.3f},{data['y']:.3f},{data['flag']:.1f},#"
            # msg = f"@{data['pan_deg']:.3f},{data['tilt_deg']:.3f},{data['flag']:.1f},#"
            if ser and ser.is_open:
                ser.write(msg.encode('utf-8'))
            
            # 打印与原始代码完全一致的输出
            print(f"{data['x']} {data['y']} {data['flag']}")
            # print(f"{data['pan_deg']:.3f},{data['tilt_deg']:.3f}, {data['flag']}")

        except queue.Empty:
            continue
    print("发送线程已停止")


# --- 3. 辅助函数 (已修改为与原始代码行为一致) ---

# task, calculate_centroid, merge_close_contours, order_points 函数保持不变...
def task(non): pass
def calculate_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0: return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
def merge_close_contours(contours, area_diff_thresh=0.1, centroid_dist_thresh=10):
    if not contours: return []
    contour_properties = []
    for c in contours:
        area = cv2.contourArea(c)
        centroid = calculate_centroid(c)
        if centroid:
            contour_properties.append({'area': area, 'centroid': centroid, 'contour': c})
    contour_properties.sort(key=lambda p: p['area'], reverse=True)
    contours = [p['contour'] for p in contour_properties]
    props = [{'area': p['area'], 'centroid': p['centroid']} for p in contour_properties]
    to_keep = [True] * len(contours)
    for i in range(len(contours)):
        if not to_keep[i]: continue
        for j in range(i + 1, len(contours)):
            if not to_keep[j]: continue
            max_area = max(props[i]['area'], props[j]['area'])
            if max_area == 0: continue
            area_diff = abs(props[i]['area'] - props[j]['area']) / max_area
            dist = math.sqrt((props[i]['centroid'][0] - props[j]['centroid'][0])**2 + (props[i]['centroid'][1] - props[j]['centroid'][1])**2)
            if area_diff < area_diff_thresh and dist < centroid_dist_thresh:
                to_keep[j] = False
    return [contours[i] for i in range(len(contours)) if to_keep[i]]
def order_points(pts):
    pts = np.array(pts, dtype="float32")
    if pts.ndim == 3 and pts.shape[1] == 1:
        pts = pts.reshape(pts.shape[0], 2)
    if len(pts) != 4: return None
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1).flatten()
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


# --- 主筛选函数 ---
def find_a4_paper_contours(contour, size_tolerance_mm=50):
    """
    使用PnP算法从潜在的四角轮廓中筛选出符合A4纸尺寸的轮廓。

    Args:
        image (np.array): 原始图像。
        potential_contours (list): 包含四个角点的潜在轮廓列表。
        size_tolerance_mm (int): 尺寸容忍度 (毫米)。

    Returns:
        list: 筛选后符合A4纸尺寸的轮廓列表，每个元素是一个numpy数组。
    """
    # 定义A4纸的三维模型点 (世界坐标系)
    object_points = np.array([
        [-A4_WIDTH_MM / 2, -A4_HEIGHT_MM / 2, 0],
        [A4_WIDTH_MM / 2, -A4_HEIGHT_MM / 2, 0],
        [A4_WIDTH_MM / 2, A4_HEIGHT_MM / 2, 0],
        [-A4_WIDTH_MM / 2, A4_HEIGHT_MM / 2, 0]
    ], dtype="float32")

    # 对角点进行排序
    ordered_pts = order_points(contour)

    # 使用PnP解算位姿
    (success, rvec, tvec) = cv2.solvePnP(object_points, ordered_pts, CAMERA_MATRIX, DIST_COEFFS)

    if success:
        # PnP解算出平移向量 (tvec)
        distance_z_mm = tvec[2][0]
        print("distance_z_mm: ",distance_z_mm)
        # 如果物体离摄像头太近或太远，可能不是A4纸
        if distance_z_mm < 100 or distance_z_mm > 2000:  # 例如，10cm到2m
            return False

        # 使用projectPoints将A4纸的角点重新投影到图像上
        # 然后计算投影后矩形的物理宽度和高度
        reprojected_points, _ = cv2.projectPoints(object_points, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
        reprojected_points = reprojected_points.reshape(-1, 2)

        # 计算投影后的物理宽度和高度（在图像中的像素距离）
        width_pixels_reprojected = np.linalg.norm(reprojected_points[1] - reprojected_points[0])
        height_pixels_reprojected = np.linalg.norm(reprojected_points[3] - reprojected_points[0])

        # 估算实际物理长宽
        estimated_width_mm = (width_pixels_reprojected * distance_z_mm) / CAMERA_MATRIX[0, 0]
        estimated_height_mm = (height_pixels_reprojected * distance_z_mm) / CAMERA_MATRIX[1, 1]

        # 检查估算的尺寸是否在容忍范围内
        is_a4_size = (
                             abs(estimated_width_mm - A4_WIDTH_MM) < size_tolerance_mm and
                             abs(estimated_height_mm - A4_HEIGHT_MM) < size_tolerance_mm
                     ) or (
                             abs(estimated_width_mm - A4_HEIGHT_MM) < size_tolerance_mm and
                             abs(estimated_height_mm - A4_WIDTH_MM) < size_tolerance_mm
                     )

        if is_a4_size:
            return True

    return False
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

    # 获取摄像头图像的中心像素点
    camera_center_x_pixel = image.shape[1] / 2
    camera_center_y_pixel = image.shape[0] / 2
    camera_center_pixel_coords = np.array([[[camera_center_x_pixel, camera_center_y_pixel]]], dtype="float32")

    camera_center_on_a4_pixels = cv2.perspectiveTransform(camera_center_pixel_coords, M)[0][0]
    # 将摄像头中心点从 A4 纸像素坐标转换为 A4 纸物理坐标 (毫米)
    camera_center_on_a4_mm_x = camera_center_on_a4_pixels[0] * PIXEL_TO_MM_RATIO
    camera_center_on_a4_mm_y = camera_center_on_a4_pixels[1] * PIXEL_TO_MM_RATIO

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

    # 将矫正后的中心点映射回原始图像
    original_center_point = cv2.perspectiveTransform(warped_center_point, Minv)
    # --- 4. 计算摄像头中心与 A4 纸中心的物理距离差值 ---
    delta_x_mm = camera_center_on_a4_mm_x - a4_center_x_mm
    delta_y_mm = camera_center_on_a4_mm_y - a4_center_y_mm  # 注意这里的Y轴，如果A4纸坐标系Y轴向下，则y_cam - y_a4_center
    # --- 4. 计算摄像头中心到矩形中心的物理距离 (毫米) ---
    distance_mm_estimated = math.sqrt(delta_x_mm**2 + delta_y_mm**2)
    # --- 5. 计算摄像头中心到矩形中心的方向角度 (度) ---
    # 使用 atan2(y, x) 计算角度。结果范围在 [-pi, pi] 或 [-180, 180] 度。
    # 角度定义：以 A4 中心为原点，正 X 轴（向右）为 0 度，逆时针为正。
    angle_radians = math.atan2(delta_y_mm, delta_x_mm)
    angle_degrees = math.degrees(angle_radians)
    # --- 计算 Pan 和 Tilt 角度 ---

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
        return image, (original_center_x, original_center_y)  # 即使超出范围，仍然返回差值

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


    return image, (original_center_x, original_center_y)


# --- 基于 PnP 的 `perspective_transform_a4` 函数 ---
def perspective_transform_a4_pnp(image, distorted_rect_coords):
    """
    使用 PnP 算法计算摄像头到 A4 纸的物理距离和 Pan/Tilt 角度。

    Args:
        image (np.array): 原始图像。
        distorted_rect_coords (list or np.array): 畸变矩形的四个角点坐标。

    Returns:
        tuple: (processed_image, pan_angle_degrees, tilt_angle_degrees, distance_mm)
               - processed_image (np.array): 绘制了信息的原始图像帧。
               - pan_angle_degrees (float): 云台水平旋转角度 (度)，正值表示向右。
               - tilt_angle_degrees (float): 云台垂直俯仰角度 (度)，正值表示向上。
               - distance_mm (float): 摄像头到A4纸的物理距离 (毫米)。
               如果 PnP 失败或点数不正确，返回 (原始图像, None, None, None)。
    """
    # --- 常量定义 ---
    # A4 纸的物理尺寸 (毫米)

    # 输入验证和点排序
    try:
        pts = np.array(distorted_rect_coords, dtype="float32")
        ordered_pts = order_points(pts)
    except ValueError as e:
        cv2.putText(image, f"点数错误或格式不正确: {e}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return image, None, None, None, None, None

    # --- 1. 定义 A4 纸的三维模型点 (世界坐标系) ---
    # 我们以 A4 纸的中心为原点 (0, 0, 0)。
    # A4_WIDTH_MM 和 A4_HEIGHT_MM 是全长，所以点坐标是 +/- 一半。
    object_points = np.array([
        [-A4_WIDTH_MM / 2, -A4_HEIGHT_MM / 2, 0],  # 左上
        [A4_WIDTH_MM / 2, -A4_HEIGHT_MM / 2, 0],  # 右上
        [A4_WIDTH_MM / 2, A4_HEIGHT_MM / 2, 0],  # 右下
        [-A4_WIDTH_MM / 2, A4_HEIGHT_MM / 2, 0]  # 左下
    ], dtype="float32")

    # --- 2. 使用 PnP 算法求解姿态 ---
    # pts 是图像中的二维点，object_points 是对应的三维点
    (success, rvec, tvec) = cv2.solvePnP(object_points, ordered_pts, CAMERA_MATRIX, DIST_COEFFS)

    if not success:
        cv2.putText(image, "PnP 算法失败！", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return image, None, None, None, None, None

    # --- 3. 从 tvec 和 rvec 提取信息 ---
    # tvec 是平移向量 [Tx, Ty, Tz]
    # Tx, Ty 是A4纸中心在摄像头坐标系中的 X, Y 坐标（毫米）
    # Tz 是A4纸到摄像头的距离（毫米）
    distance_x_mm = tvec[0][0]
    distance_y_mm = tvec[1][0]
    distance_z_mm = tvec[2][0]

    # 计算总距离 (欧几里得距离)
    distance_mm = math.sqrt(distance_x_mm ** 2 + distance_y_mm ** 2 + distance_z_mm ** 2)

    # --- 4. 从平移向量计算 Pan 和 Tilt 角度 ---
    # Pan 角度：atan(X / Z)
    # Tilt 角度：atan(Y / Z)
    # 这里我们假设云台的 Pan 轴与摄像头的 X 轴对齐，Tilt 轴与 Y 轴对齐。
    # 实际应用中可能需要根据云台的安装方式进行调整。
    pan_angle_radians = math.atan2(distance_x_mm, distance_z_mm)
    tilt_angle_radians = math.atan2(distance_y_mm, distance_z_mm)

    pan_angle_degrees = math.degrees(pan_angle_radians)
    tilt_angle_degrees = math.degrees(tilt_angle_radians)

    # --- 绘制辅助线和信息 ---
    # (此处省略，请从之前的代码中复制绘制和文本显示部分)
    # ...

    # 将3D坐标投影回图像，用于绘制辅助线，例如，绘制一个立方体
    axis = np.float32([[A4_WIDTH_MM / 2, -A4_HEIGHT_MM / 2, 0],
                       [-A4_WIDTH_MM / 2, -A4_HEIGHT_MM / 2, 0],
                       [-A4_WIDTH_MM / 2, A4_HEIGHT_MM / 2, 0],
                       [A4_WIDTH_MM / 2, A4_HEIGHT_MM / 2, 0]]).reshape(-1, 3)
    imgpts, jac = cv2.projectPoints(axis, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
    imgpts = imgpts.reshape(-1, 2).astype(int)
    cv2.polylines(image, [imgpts], True, (0, 255, 0), 2)  # 绿色矩形

    # 绘制 A4 纸中心
    center_3d = np.array([[0, 0, 0]], dtype="float32")
    center_2d, jac = cv2.projectPoints(center_3d, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
    center_2d = center_2d.reshape(-1, 2).astype(int)
    cv2.circle(image, (center_2d[0][0], center_2d[0][1]), 1, (255, 255, 0), -1)  # 黄色 A4 中心点

    # 绘制摄像头中心点
    camera_center_pixel_on_original = (int(image.shape[1] / 2), int(image.shape[0] / 2))
    cv2.circle(image, camera_center_pixel_on_original, 1, (255, 0, 0), -1)  # 蓝色摄像头中心

    # 显示信息
    cv2.putText(image, f"Pan: {pan_angle_degrees:.1f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255),
                2)
    cv2.putText(image, f"Tilt: {tilt_angle_degrees:.1f} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 255), 2)
    cv2.putText(image, f"distance (Z): {distance_z_mm:.1f} mm", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(image, f"X Delta: {distance_x_mm:.1f} mm", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(image, f"Y Delta: {distance_y_mm:.1f} mm", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    # 返回处理后的图像，以及计算出的 Pan 和 Tilt 角度
    return image, pan_angle_degrees, tilt_angle_degrees, distance_mm, center_2d[0][0], center_2d[0][1]


# --- 4. 主程序 ---
if __name__ == "__main__":
    port = '/dev/ttyAMA2'
    baudrate = 115200
    ser = None
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        print(f"串口 {port} 已打开，波特率 {baudrate}")
        ser.write("begin".encode("utf-8"))
    except serial.SerialException as e:
        print(f"无法打开串口 {port}: {e}. 程序将在无串口模式下运行。")

    width, height = 640, 480#640, 480    1900, 1080
    light_x, light_y = 319,243#319,243
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误：无法打开摄像头。")
        exit()

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 60)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,50)

    # GUI窗口和Trackbar (标签与原始代码一致)
    cv2.namedWindow('Trackbar', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('H_Min','Trackbar',trackbar_values['lower'][0],180,task)
    cv2.createTrackbar('S_Min','Trackbar',trackbar_values['lower'][1],255,task)
    cv2.createTrackbar('V_Min','Trackbar',trackbar_values['lower'][2],255,task)
    cv2.createTrackbar('H_Max','Trackbar',trackbar_values['upper'][0],180,task)
    cv2.createTrackbar('S_Max','Trackbar',trackbar_values['upper'][1],255,task)
    cv2.createTrackbar('V_Max','Trackbar',trackbar_values['upper'][2],255,task)
    cv2.createTrackbar('ok','Trackbar',0,1,task)
    # 创建并启动线程
    reader = threading.Thread(target=read_thread_func, args=(cap,))
    processor = threading.Thread(target=process_thread_func)
    sender = threading.Thread(target=send_thread_func, args=(ser,))
    
    reader.start()
    processor.start()
    sender.start()

    # 主循环 (GUI和控制)
    while True:
        # 更新处理线程所需的Trackbar值
        trackbar_values['lower'][0] = cv2.getTrackbarPos('H_Min', 'Trackbar')
        trackbar_values['lower'][1] = cv2.getTrackbarPos('S_Min', 'Trackbar')
        trackbar_values['lower'][2] = cv2.getTrackbarPos('V_Min', 'Trackbar')
        trackbar_values['upper'][0] = cv2.getTrackbarPos('H_Max', 'Trackbar')
        trackbar_values['upper'][1] = cv2.getTrackbarPos('S_Max', 'Trackbar')
        trackbar_values['upper'][2] = cv2.getTrackbarPos('V_Max', 'Trackbar')
        if cv2.getTrackbarPos('ok', 'Trackbar'):
            save_thresholds()
        # 从共享变量中获取并显示图像
        with display_lock:
            if display_frame is not None:
                cv2.imshow('Detected Rectangles', display_frame)
                cv2.imshow('inRange_mask', display_inRange_mask)
                # cv2.imshow('display_inRange_edges',display_inRange_edges)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            print("正在退出...")
            running_event.clear()
            break

    # 清理工作
    reader.join()
    processor.join()
    sender.join()
    cap.release()
    if ser and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()
    print("程序已成功关闭。")