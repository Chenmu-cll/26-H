# -*- coding: utf-8 -*-
'''
Script: deploy_det_video.py
脚本名称：deploy_det_video.py

Description:
    This script runs a real-time object detection application on an embedded device.
    It uses a pipeline to capture video frames, performs inference using a pre-trained Kmodel,
    and displays the detection results (bounding boxes, class labels) on screen.

    The model configuration is loaded from the Canaan online training platform via a JSON config file.

脚本说明：
    本脚本在嵌入式设备上运行实时目标检测应用。它通过捕获视频帧，使用预训练的 Kmodel 进行推理，并在屏幕上显示检测结果（边界框、类别标签）。

    模型配置文件通过 Canaan 在线训练平台从 JSON 文件加载
    新增功能：
    - 垂直中心线：将画面分为左右两半
    - UART串口输出：钢球中心X偏移量（左负右正），目标丢失发送999

'''


import os, gc, time
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *
from media.sensor import *
from machine import UART
from machine import FPIOA

# ==================== 硬件串口初始化 ====================
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD)
fpioa.set_function(10, fpioa.UART1_RXD)

uart1 = UART(
    UART.UART1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)

# Set display mode: options are 'hdmi', 'lcd', 'lt9611', 'st7701', 'hx8399'
# 'hdmi' defaults to 'lt9611' (1920x1080); 'lcd' defaults to 'st7701' (800x480)
display_mode = "lt9611"

# Define the input size for the RGB888P video frames
rgb888p_size = [1920, 1080]

# Set root directory path for model and config
root_path = "/sdcard/mp_deployment_source/"

# Load deployment configuration
deploy_conf = read_json(root_path + "/deploy_config.json")
kmodel_path = root_path + deploy_conf["kmodel_path"]              # KModel path
labels = deploy_conf["categories"]                                # Label list
confidence_threshold = deploy_conf["confidence_threshold"]        # Confidence threshold
nms_threshold = deploy_conf["nms_threshold"]                      # NMS threshold
model_input_size = deploy_conf["img_size"]                        # Model input size
nms_option = deploy_conf["nms_option"]                            # NMS strategy
model_type = deploy_conf["model_type"]                            # Detection model type
anchors = []
if model_type == "AnchorBaseDet":
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

# Inference configuration
inference_mode = "video"                                          # Inference mode: 'video'
debug_mode = 0                                                    # Debug mode flag

# Create and initialize the video/display pipeline
pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
pl.create(sensor=Sensor(id=2, width=1920, height=1080))  # CSI2摄像头，全高清采集
display_size = pl.get_display_size()

# 画面垂直中心线X坐标（将画面分为左右两半）
CENTER_X = display_size[0] // 2

# Initialize object detection application
det_app = DetectionApp(inference_mode,kmodel_path,labels,model_input_size,anchors,model_type,confidence_threshold,nms_threshold,rgb888p_size,display_size,debug_mode=debug_mode)

# Configure preprocessing for the model
det_app.config_preprocess()


# ==================== ROI 动态跟踪配置 ====================
# 搜索ROI（display坐标系）：未找到球时的大范围搜索区域
SEARCH_ROI = (320, 100, 1280, 880)
# 跟踪窗口大小（display坐标系）：找到球后，ROI以此大小跟随球移动
TRACK_W, TRACK_H = 200, 200

# 当前使用的ROI（display坐标系），初始为搜索ROI
cur_roi = list(SEARCH_ROI)  # [x, y, w, h]，动态更新


def roi_to_rgb(roi):
    """将display坐标系的ROI换算到rgb888p坐标系"""
    return (
        roi[0] * rgb888p_size[0] // display_size[0],
        roi[1] * rgb888p_size[1] // display_size[1],
        roi[2] * rgb888p_size[0] // display_size[0],
        roi[3] * rgb888p_size[1] // display_size[1],
    )


def is_inside_roi(box):
    """判断检测框中心是否在当前ROI区域内（box使用rgb888p坐标）"""
    cx = (box[0] + box[2]) // 2
    cy = (box[1] + box[3]) // 2
    rx, ry, rw, rh = roi_to_rgb(cur_roi)
    return rx <= cx <= rx + rw and ry <= cy <= ry + rh


def update_roi(ball_cx_disp, ball_cy_disp):
    """根据钢球位置更新ROI，使其跟随钢球移动"""
    cur_roi[0] = max(0, ball_cx_disp - TRACK_W // 2)
    cur_roi[1] = max(0, ball_cy_disp - TRACK_H // 2)
    cur_roi[2] = TRACK_W
    cur_roi[3] = TRACK_H
    # 边界裁剪，防止ROI超出画面
    if cur_roi[0] + cur_roi[2] > display_size[0]:
        cur_roi[0] = display_size[0] - cur_roi[2]
    if cur_roi[1] + cur_roi[3] > display_size[1]:
        cur_roi[1] = display_size[1] - cur_roi[3]


def reset_roi():
    """丢失目标时，重置为搜索ROI"""
    cur_roi[0] = SEARCH_ROI[0]
    cur_roi[1] = SEARCH_ROI[1]
    cur_roi[2] = SEARCH_ROI[2]
    cur_roi[3] = SEARCH_ROI[3]


def find_best_ball(res):
    """从检测结果中找到当前ROI区域内置信度最高的钢球"""
    if not res or not res.get("boxes"):
        return None
    best_idx = -1
    best_conf = 0.0
    for i, score in enumerate(res["scores"]):
        if score > best_conf and is_inside_roi(res["boxes"][i]):
            best_conf = score
            best_idx = i
    if best_idx >= 0:
        boxes = res["boxes"][best_idx]
        return {
            "x1": boxes[0],
            "y1": boxes[1],
            "x2": boxes[2],
            "y2": boxes[3],
            "score": best_conf,
            "class_id": res["idx"][best_idx],
        }
    return None


def draw_roi(img):
    """在OSD图像上绘制当前ROI区域"""
    x, y, w, h = cur_roi
    img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=1)


def draw_center_line(img, cx):
    """在OSD图像上绘制垂直中心线，将画面分为左右两半"""
    color = (0, 255, 0)  # 绿色
    img.draw_line(cx, 0, cx, display_size[1], color=color, thickness=1)


# FPS tracking
last_fps_time = time.ticks_ms()
fps = 0.0
frame_count = 0

# Main loop: capture, run inference, display results
while True:
    with ScopedTiming("total", 1):
        # FPS calculation (update every 500ms)
        frame_count += 1
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_fps_time)
        if dt >= 500:
            fps = frame_count * 1000.0 / dt
            frame_count = 0
            last_fps_time = now

        img = pl.get_frame()                          # Capture current frame
        res = det_app.run(img)                        # Run inference

        # 清除上一帧OSD绘制内容
        pl.osd_img.clear()

        # 绘制垂直中心线（分左右半区）
        draw_center_line(pl.osd_img, CENTER_X)

        # 绘制ROI检测区域
        draw_roi(pl.osd_img)

        # 只识别置信度最高的那一个钢球
        best_ball = find_best_ball(res)

        if best_ball is not None:
            # 坐标缩放到显示坐标系
            x1 = best_ball["x1"] * display_size[0] // rgb888p_size[0]
            y1 = best_ball["y1"] * display_size[1] // rgb888p_size[1]
            x2 = best_ball["x2"] * display_size[0] // rgb888p_size[0]
            y2 = best_ball["y2"] * display_size[1] // rgb888p_size[1]

            # 钢球中心坐标
            ball_cx_disp = (x1 + x2) // 2
            ball_cy_disp = (y1 + y2) // 2

            # X偏移量：左负右正
            off_x = ball_cx_disp - CENTER_X

            # 只绘制这一个钢球的检测框 + 中心十字
            pl.osd_img.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color=(0, 255, 0), thickness=2)
            pl.osd_img.draw_cross(ball_cx_disp, ball_cy_disp, color=(255, 0, 0), size=10, thickness=2)

            # ROI跟随钢球移动
            update_roi(ball_cx_disp, ball_cy_disp)

            # UART发送: 格式 origin:(x)\r\n
            uart_msg = "origin:({})\r\n".format(off_x)
            uart1.write(uart_msg)

            # IDE终端打印
            print(uart_msg, end='')

            # 屏幕显示X偏移
            info = "X-off:{:+d}".format(off_x)
            pl.osd_img.draw_string_advanced(5, 25, 16, info, color=(255, 255, 0))
        else:
            # 丢失目标，ROI重置为搜索区域
            reset_roi()

            lost_msg = "origin:(999)\r\n"
            uart1.write(lost_msg)
            print(lost_msg, end='')

            info = "SEARCHING..."
            pl.osd_img.draw_string_advanced(5, 25, 16, info, color=(255, 0, 0))

        # 左上角显示帧率
        fps_str = "FPS:{:.1f}".format(fps)
        pl.osd_img.draw_string_advanced(5, 5, 16, fps_str, color=(0, 255, 0))

        pl.show_image()                               # Show result on display
        gc.collect()                                  # Run garbage collection

# Cleanup: These lines will only run if the loop is interrupted (e.g., by an IDE break or external interruption)
det_app.deinit()                                      # De-initialize detection app
pl.destroy()                                          # Destroy pipeline instance
