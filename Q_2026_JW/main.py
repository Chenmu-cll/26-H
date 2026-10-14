"""K230 ball detector with filtered ASCII UART position output.

UART protocol (UART1, 115200 8N1):
    origin:(<x_offset>)\r\n
``x_offset`` is the predicted ball centre relative to the image centre;
``origin:(999)`` means that no valid track is currently available.
"""

import gc
import os
import time
import utime
import _thread
import ujson
import aicube
import image
import nncase_runtime as nn
import ulab.numpy as np

from machine import FPIOA, UART
from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbBuzzer import YbBuzzer
from ybUtils.YbRGB import YbRGB


# -------------------- Hardware and model configuration --------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
OUT_RGB888P_WIDTH = 640
OUT_RGB888P_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2

# Keep the UART electrical setup and ASCII line protocol from Base/main.py.
UART_BAUDRATE = 115200
UART_TX_PIN = 9
UART_RX_PIN = 10
UART_INVALID_OFFSET = 999

ROOT_PATH = "/sdcard/mp_deployment_source/"
CONFIG_PATH = ROOT_PATH + "deploy_config.json"
DEBUG_MODE = 0
IDE_INTERRUPT_WAIT_MS = 1500
STARTUP_BEEP_FREQUENCY_HZ = 2000
STARTUP_BEEP_VOLUME_PERCENT = 50
STARTUP_BEEP_SECONDS = 0.7

DRAW_COLOR = (220, 20, 60)
SPEED_HISTORY_LENGTH = 3
MIN_SPEED_SAMPLES = 2
VELOCITY_EMA_ENABLED = True
VELOCITY_EMA_ALPHA = 0.65

STATIC_DEADBAND_PX = 7
DIRECTION_CONFIRM_FRAMES = 3
MOVING_RETURN_STATIC_FRAMES = 5
MAX_HOLD_EMPTY_FRAMES = 4
EMA_ENABLED = True
EMA_ALPHA = 0.95

YOLO_PERIOD_MS = 25
PREDICTION_PERIOD_MS = 10
DRAW_PERIOD_MS = 100
PREDICTION_STALE_TIMEOUT_MS = (MAX_HOLD_EMPTY_FRAMES + 1) * YOLO_PERIOD_MS
RATE_MEASURE_WINDOW_MS = 1000

KALMAN_ACCELERATION_NOISE = 160000.0
KALMAN_POSITION_MEASUREMENT_NOISE = 16.0
KALMAN_VELOCITY_MEASUREMENT_NOISE = 900.0
KALMAN_INITIAL_POSITION_VARIANCE = 400.0
KALMAN_INITIAL_VELOCITY_VARIANCE = 160000.0
KALMAN_MAX_PREDICT_STEP_MS = PREDICTION_STALE_TIMEOUT_MS

PERFORMANCE_PROFILE_ENABLED = False
PERFORMANCE_REPORT_INTERVAL = 30


def read_deploy_config(path=CONFIG_PATH):
    with open(path, "r") as json_file:
        return ujson.load(json_file)


def wait_for_ide_interrupt():
    """Leave a 1500 ms interruptible window before initialising hardware."""
    print("Waiting 1500 ms for IDE interrupt...")
    start_ms = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_ms) < IDE_INTERRUPT_WAIT_MS:
        os.exitpoint()
        time.sleep_ms(20)

    buzzer = YbBuzzer()
    buzzer.on(STARTUP_BEEP_FREQUENCY_HZ, STARTUP_BEEP_VOLUME_PERCENT,
              STARTUP_BEEP_SECONDS)
    buzzer.off()


def init_board_peripherals():
    """Turn off RGB LED and initialise Base/main.py's UART1 settings."""
    rgb = YbRGB()
    rgb.show_rgb((0, 0, 0))

    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, fpioa.UART1_TXD, ie=0, oe=1)
    fpioa.set_function(UART_RX_PIN, fpioa.UART1_RXD, ie=1, oe=0)
    return UART(UART.UART1, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)


def write_origin(uart, center_x, velocity=None):
    """Write one Base-compatible ASCII result line to UART."""
    if center_x is None:
        uart.write("origin:(%d) v:(---)\r\n" % UART_INVALID_OFFSET)
    else:
        v_str = ("%+d" % velocity) if velocity is not None else "---"
        uart.write("origin:(%d) v:(%s)\r\n" % (center_x - CENTER_X, v_str))


def select_ball_center(det_boxes):
    """Return the horizontal centre of the highest-confidence ball box."""
    if not det_boxes:
        return None

    best_box = det_boxes[0]
    for det_box in det_boxes[1:]:
        if det_box[1] > best_box[1]:
            best_box = det_box

    center_x = (best_box[2] + best_box[4]) / 2
    if center_x < 0:
        center_x = 0
    elif center_x > FRAME_WIDTH:
        center_x = FRAME_WIDTH
    return int(center_x + 0.5)


def calculate_signed_velocity(samples):
    """Calculate signed horizontal velocity from position/time samples."""
    first_x, first_time = samples[0]
    last_x, last_time = samples[-1]
    elapsed_ms = utime.ticks_diff(last_time, first_time)
    if elapsed_ms <= 0:
        return 0

    velocity = (last_x - first_x) * 1000 / elapsed_ms
    if velocity > 0xFFFF:
        return 0xFFFF
    if velocity < -0xFFFF:
        return -0xFFFF
    if velocity >= 0:
        return int(velocity + 0.5)
    return int(velocity - 0.5)


class VelocityTracker:
    """Keep filtered positions and derive the signed velocity observation."""
    def __init__(self):
        self.samples = []
        self.filtered_signed_velocity = None
        self.last_signed_velocity = None

    def reset(self):
        self.samples = []
        self.filtered_signed_velocity = None
        self.last_signed_velocity = None

    def update(self, filtered_center_x, timestamp_ms):
        self.samples.append((filtered_center_x, timestamp_ms))
        if len(self.samples) > SPEED_HISTORY_LENGTH:
            self.samples.pop(0)
        if len(self.samples) < MIN_SPEED_SAMPLES:
            return None

        raw_signed_velocity = calculate_signed_velocity(self.samples)
        if not VELOCITY_EMA_ENABLED or self.filtered_signed_velocity is None:
            self.filtered_signed_velocity = raw_signed_velocity
        else:
            self.filtered_signed_velocity = (
                VELOCITY_EMA_ALPHA * raw_signed_velocity +
                (1 - VELOCITY_EMA_ALPHA) * self.filtered_signed_velocity)
        self.last_signed_velocity = int(self.filtered_signed_velocity +
                                        (0.5 if self.filtered_signed_velocity >= 0
                                         else -0.5))
        return abs(self.last_signed_velocity)


class HysteresisStateFilter:
    """Reject static jitter and confirm sustained movement before tracking it."""
    STATIC = 0
    MOVING = 1
    LOST = 2

    def __init__(self):
        self.state = self.LOST
        self.filtered_x = None
        self.last_raw_x = None
        self.candidate_direction = 0
        self.direction_count = 0
        self.still_count = 0
        self.empty_frames = 0

    def _start_track(self, center_x):
        self.state = self.STATIC
        self.filtered_x = center_x
        self.last_raw_x = center_x
        self.candidate_direction = 0
        self.direction_count = 0
        self.still_count = 0

    def _apply_ema(self, center_x):
        if not EMA_ENABLED:
            return center_x
        return int(EMA_ALPHA * center_x +
                   (1 - EMA_ALPHA) * self.filtered_x + 0.5)

    def _enter_static(self, center_x):
        self.state = self.STATIC
        self.filtered_x = center_x
        self.candidate_direction = 0
        self.direction_count = 0
        self.still_count = 0

    def update(self, center_x):
        self.empty_frames = 0
        if self.state == self.LOST:
            self._start_track(center_x)
            return self.filtered_x

        if self.state == self.STATIC:
            displacement = center_x - self.filtered_x
            if abs(displacement) <= STATIC_DEADBAND_PX:
                self.candidate_direction = 0
                self.direction_count = 0
            else:
                direction = 1 if displacement > 0 else -1
                if direction == self.candidate_direction:
                    self.direction_count += 1
                else:
                    self.candidate_direction = direction
                    self.direction_count = 1
                if self.direction_count >= DIRECTION_CONFIRM_FRAMES:
                    self.state = self.MOVING
                    self.still_count = 0
                    self.filtered_x = self._apply_ema(center_x)
        else:
            frame_delta = center_x - self.last_raw_x
            if abs(frame_delta) <= STATIC_DEADBAND_PX:
                self.still_count += 1
            else:
                self.still_count = 0
            self.filtered_x = self._apply_ema(center_x)
            if self.still_count >= MOVING_RETURN_STATIC_FRAMES:
                self._enter_static(center_x)

        self.last_raw_x = center_x
        return self.filtered_x

    def handle_empty_frame(self):
        if self.state == self.LOST:
            return None
        self.empty_frames += 1
        if self.empty_frames <= MAX_HOLD_EMPTY_FRAMES:
            return self.filtered_x

        self.state = self.LOST
        self.last_raw_x = None
        self.candidate_direction = 0
        self.direction_count = 0
        self.still_count = 0
        return None


class KalmanPredictor:
    """Two-state constant-velocity predictor for the latest ball position."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.position = 0.0
        self.velocity = 0.0
        self.p00 = KALMAN_INITIAL_POSITION_VARIANCE
        self.p01 = 0.0
        self.p11 = KALMAN_INITIAL_VELOCITY_VARIANCE
        self.last_time_ms = None
        self.ready = False

    def _predict_to(self, timestamp_ms):
        if self.last_time_ms is None:
            self.last_time_ms = timestamp_ms
            return
        elapsed_ms = utime.ticks_diff(timestamp_ms, self.last_time_ms)
        if elapsed_ms <= 0:
            return
        if elapsed_ms > KALMAN_MAX_PREDICT_STEP_MS:
            elapsed_ms = KALMAN_MAX_PREDICT_STEP_MS
        dt = elapsed_ms / 1000.0

        q00 = KALMAN_ACCELERATION_NOISE * dt * dt * dt * dt / 4
        q01 = KALMAN_ACCELERATION_NOISE * dt * dt * dt / 2
        q11 = KALMAN_ACCELERATION_NOISE * dt * dt
        old_p00, old_p01, old_p11 = self.p00, self.p01, self.p11
        self.position += self.velocity * dt
        self.p00 = old_p00 + 2 * dt * old_p01 + dt * dt * old_p11 + q00
        self.p01 = old_p01 + dt * old_p11 + q01
        self.p11 = old_p11 + q11
        self.last_time_ms = timestamp_ms

    def _update_position(self, measured_position):
        innovation = measured_position - self.position
        denominator = self.p00 + KALMAN_POSITION_MEASUREMENT_NOISE
        if denominator <= 0:
            return
        k0, k1 = self.p00 / denominator, self.p01 / denominator
        old_p00, old_p01 = self.p00, self.p01
        self.position += k0 * innovation
        self.velocity += k1 * innovation
        self.p00 = (1 - k0) * old_p00
        self.p01 = (1 - k0) * old_p01
        self.p11 -= k1 * old_p01

    def _update_velocity(self, measured_velocity):
        innovation = measured_velocity - self.velocity
        denominator = self.p11 + KALMAN_VELOCITY_MEASUREMENT_NOISE
        if denominator <= 0:
            return
        k0, k1 = self.p01 / denominator, self.p11 / denominator
        old_p01, old_p11 = self.p01, self.p11
        self.position += k0 * innovation
        self.velocity += k1 * innovation
        self.p00 -= k0 * old_p01
        self.p01 = (1 - k1) * old_p01
        self.p11 = (1 - k1) * old_p11

    def observe(self, yolo_position, filtered_velocity, timestamp_ms):
        if not self.ready:
            self.position = float(yolo_position)
            self.velocity = float(filtered_velocity)
            self.last_time_ms = timestamp_ms
            self.ready = True
            return
        self._predict_to(timestamp_ms)
        self._update_position(yolo_position)
        self._update_velocity(filtered_velocity)

    def predict(self, timestamp_ms):
        if not self.ready:
            return None
        self._predict_to(timestamp_ms)
        position = int(self.position + 0.5)
        if position < 0:
            position = 0
        elif position > FRAME_WIDTH:
            position = FRAME_WIDTH
        return position


class RateMeter:
    def __init__(self, window_ms):
        self.window_ms = window_ms
        self.window_start_ms = None
        self.count = 0
        self.hz = 0.0

    def mark(self, timestamp_ms):
        if self.window_start_ms is None:
            self.window_start_ms = timestamp_ms
            return self.hz
        self.count += 1
        elapsed_ms = utime.ticks_diff(timestamp_ms, self.window_start_ms)
        if elapsed_ms >= self.window_ms:
            self.hz = self.count * 1000.0 / elapsed_ms
            self.window_start_ms = timestamp_ms
            self.count = 0
        return self.hz


class FrameTimingProfiler:
    """Optional windowed profiling; disabled by default to protect frame rate."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.frame_count = 0
        self.capture_total_ms = 0
        self.yolo_total_ms = 0
        self.draw_total_ms = 0
        self.algorithm_total_ms = 0
        self.frame_total_ms = 0

    def record(self, capture_ms, yolo_ms, draw_ms, algorithm_ms, total_ms):
        if not PERFORMANCE_PROFILE_ENABLED:
            return None
        self.frame_count += 1
        self.capture_total_ms += capture_ms
        self.yolo_total_ms += yolo_ms
        self.draw_total_ms += draw_ms
        self.algorithm_total_ms += algorithm_ms
        self.frame_total_ms += total_ms
        if self.frame_count < PERFORMANCE_REPORT_INTERVAL:
            return None
        count = self.frame_count
        summary = (count, self.capture_total_ms / count,
                   self.yolo_total_ms / count, self.draw_total_ms / count,
                   self.algorithm_total_ms / count,
                   self.frame_total_ms / count)
        self.reset()
        return summary


def draw_detection_overlay(osd_img, det_boxes, labels, status=None):
    osd_img.clear()
    osd_img.draw_line(CENTER_X, 0, CENTER_X, DISPLAY_HEIGHT, color=DRAW_COLOR)
    if det_boxes:
        for det_box in det_boxes:
            x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
            draw_x = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
            draw_y = int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT)
            draw_w = int(float(x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
            draw_h = int(float(y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT)
            osd_img.draw_rectangle(draw_x, draw_y, draw_w, draw_h,
                                   color=DRAW_COLOR)
            label = labels[det_box[0]]
            osd_img.draw_string_advanced(draw_x, draw_y - 50, 32,
                                         label + " " + str(round(det_box[1], 2)),
                                         color=DRAW_COLOR)
    if status is not None:
        yolo_hz, output_hz, _, offset, signed_v = status
        offset_str = ("%+d" % offset) if offset is not None else "---"
        v_str = ("%+d" % signed_v) if signed_v is not None else "---"
        osd_img.draw_string_advanced(10, 10, 24,
                                     "YOLO:%.1fHz OUT:%.1fHz err:%spx V:%spx/s" %
                                     (yolo_hz, output_hz, offset_str, v_str),
                                     color=(255, 255, 0))
    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

def create_preprocessor(deploy_conf):
    model_width, model_height = deploy_conf["img_size"]
    ratio = min(float(model_width) / OUT_RGB888P_WIDTH,
                float(model_height) / OUT_RGB888P_HEIGHT)
    new_w = int(ratio * OUT_RGB888P_WIDTH)
    new_h = int(ratio * OUT_RGB888P_HEIGHT)
    dw = float(model_width - new_w) / 2
    dh = float(model_height - new_h) / 2
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw - 0.1))

    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
                   np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0,
                       [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear,
                          nn.interp_mode.half_pixel)
    builder = ai2d.build([1, 3, OUT_RGB888P_HEIGHT, OUT_RGB888P_WIDTH],
                         [1, 3, model_height, model_width])
    return builder, model_width, model_height


def post_process(results, deploy_conf):
    common_args = (results[0], results[1], results[2], deploy_conf["img_size"],
                   [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGHT], [8, 16, 32],
                   deploy_conf["num_classes"], deploy_conf["confidence_threshold"],
                   deploy_conf["nms_threshold"])
    if deploy_conf["model_type"] == "AnchorBaseDet":
        anchors = (deploy_conf["anchors"][0] + deploy_conf["anchors"][1] +
                   deploy_conf["anchors"][2])
        return aicube.anchorbasedet_post_process(
            *(common_args + (anchors, deploy_conf["nms_option"])))
    if deploy_conf["model_type"] == "GFLDet":
        return aicube.gfldet_post_process(
            *(common_args + (deploy_conf["nms_option"],)))
    return aicube.anchorfreedet_post_process(
        *(common_args + (deploy_conf["nms_option"],)))


def configure_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGHT,
                         chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    return sensor


class PredictionMailbox:
    """Safely share Kalman state between the YOLO loop and UART worker."""
    def __init__(self):
        self.lock = _thread.allocate_lock()
        self.kalman = KalmanPredictor()
        self.last_measurement_ms = None
        self.instant_pixel_speed = 0
        self.yolo_rate = RateMeter(RATE_MEASURE_WINDOW_MS)
        self.output_rate = RateMeter(RATE_MEASURE_WINDOW_MS)
        self.last_offset = None
        self.last_signed_velocity = 0

    def mark_yolo(self, timestamp_ms):
        self.lock.acquire()
        try:
            self.yolo_rate.mark(timestamp_ms)
        finally:
            self.lock.release()

    def reset_tracking(self):
        self.lock.acquire()
        try:
            self.kalman.reset()
            self.last_measurement_ms = None
            self.instant_pixel_speed = 0
            self.last_signed_velocity = 0
        finally:
            self.lock.release()

    def accept_measurement(self, yolo_center_x, filtered_signed_speed,
                           timestamp_ms):
        self.lock.acquire()
        try:
            self.kalman.observe(yolo_center_x, filtered_signed_speed,
                                timestamp_ms)
            self.last_measurement_ms = timestamp_ms
            self.instant_pixel_speed = abs(filtered_signed_speed)
            self.last_signed_velocity = filtered_signed_speed
        finally:
            self.lock.release()

    def next_center(self, timestamp_ms):
        self.lock.acquire()
        try:
            self.output_rate.mark(timestamp_ms)
            if (self.last_measurement_ms is None or
                    utime.ticks_diff(timestamp_ms, self.last_measurement_ms) >
                    PREDICTION_STALE_TIMEOUT_MS):
                self.last_offset = None
                return None
            center = self.kalman.predict(timestamp_ms)
            self.last_offset = (center - CENTER_X) if center is not None else None
            return center
        finally:
            self.lock.release()

    def next_output(self, timestamp_ms):
        self.lock.acquire()
        try:
            self.output_rate.mark(timestamp_ms)
            if (self.last_measurement_ms is None or
                    utime.ticks_diff(timestamp_ms, self.last_measurement_ms) >
                    PREDICTION_STALE_TIMEOUT_MS):
                self.last_offset = None
                return None, None
            center = self.kalman.predict(timestamp_ms)
            self.last_offset = (center - CENTER_X) if center is not None else None
            return center, self.last_signed_velocity
        finally:
            self.lock.release()

    def overlay_status(self):
        self.lock.acquire()
        try:
            return (self.yolo_rate.hz, self.output_rate.hz,
                    self.instant_pixel_speed, self.last_offset,
                    self.last_signed_velocity)
        finally:
            self.lock.release()


class PredictionOutputWorker:
    """Emit Base-format ASCII predictions at the original 10 ms cadence."""
    def __init__(self, uart, mailbox):
        self.uart = uart
        self.mailbox = mailbox
        self.running = False
        self.thread_over = True

    def start(self):
        self.running = True
        self.thread_over = False
        _thread.start_new_thread(self._run, ())

    def stop(self):
        if not self.running:
            return
        self.running = False
        while not self.thread_over:
            time.sleep_ms(5)

    def _run(self):
        next_tick_ms = utime.ticks_ms()
        try:
            while self.running:
                now_ms = utime.ticks_ms()
                wait_ms = utime.ticks_diff(next_tick_ms, now_ms)
                if wait_ms > 0:
                    time.sleep_ms(wait_ms)
                    continue
                center, velocity = self.mailbox.next_output(now_ms)
                write_origin(self.uart, center, velocity)
                next_tick_ms = utime.ticks_add(next_tick_ms,
                                                PREDICTION_PERIOD_MS)
                if utime.ticks_diff(now_ms, next_tick_ms) > 0:
                    next_tick_ms = utime.ticks_add(now_ms,
                                                    PREDICTION_PERIOD_MS)
        finally:
            self.thread_over = True


def elapsed_ms(start_ns):
    return (time.time_ns() - start_ns) / 1000000


def report_frame_timing(profiler, capture_ms, yolo_ms, draw_ms,
                        algorithm_ms, frame_start_ns):
    summary = profiler.record(capture_ms, yolo_ms, draw_ms, algorithm_ms,
                              elapsed_ms(frame_start_ns))
    if summary is not None:
        count, capture, yolo, draw, algorithm, total = summary
        print("Timing avg/%d: capture=%.2fms yolo=%.2fms draw=%.2fms "
              "algorithm=%.2fms total=%.2fms" %
              (count, capture, yolo, draw, algorithm, total))


def detection():
    print("det_infer start")
    sensor = None
    uart = None
    output_worker = None
    ai2d_input_tensor = None
    ai2d_output_tensor = None

    try:
        uart = init_board_peripherals()
        deploy_conf = read_deploy_config(CONFIG_PATH)
        labels = deploy_conf["categories"]
        ai2d_builder, model_width, model_height = create_preprocessor(deploy_conf)

        kpu = nn.kpu()
        kpu.load_kmodel(ROOT_PATH + deploy_conf["kmodel_path"])
        sensor = configure_sensor()
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT,
                     to_ide=True)
        osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
        MediaManager.init()
        sensor.run()

        data = np.ones((1, 3, model_height, model_width), dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        position_filter = HysteresisStateFilter()
        velocity_tracker = VelocityTracker()
        timing_profiler = FrameTimingProfiler()
        prediction_mailbox = PredictionMailbox()
        output_worker = PredictionOutputWorker(uart, prediction_mailbox)
        output_worker.start()
        next_yolo_ms = utime.ticks_ms()
        last_draw_ms = utime.ticks_add(next_yolo_ms, -DRAW_PERIOD_MS)

        while True:
            os.exitpoint()
            wait_ms = utime.ticks_diff(next_yolo_ms, utime.ticks_ms())
            if wait_ms > 0:
                time.sleep_ms(wait_ms)
            frame_start_ns = time.time_ns()
            capture_start_ns = time.time_ns()
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            capture_ms = elapsed_ms(capture_start_ns)
            yolo_ms = 0
            draw_ms = 0
            det_boxes = None

            if rgb888p_img.format() != image.RGBP888:
                algorithm_start_ns = time.time_ns()
                if position_filter.handle_empty_frame() is None:
                    velocity_tracker.reset()
                    prediction_mailbox.reset_tracking()
                algorithm_ms = elapsed_ms(algorithm_start_ns)
                prediction_mailbox.mark_yolo(utime.ticks_ms())
            else:
                yolo_start_ns = time.time_ns()
                ai2d_input_tensor = nn.from_numpy(rgb888p_img.to_numpy_ref())
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                kpu.set_input_tensor(0, ai2d_output_tensor)
                kpu.run()

                results = []
                for index in range(kpu.outputs_size()):
                    output_tensor = kpu.get_output_tensor(index)
                    result = output_tensor.to_numpy()
                    result = result.reshape((result.shape[0] * result.shape[1] *
                                             result.shape[2] * result.shape[3]))
                    del output_tensor
                    results.append(result)
                gc.collect()
                det_boxes = post_process(results, deploy_conf)
                yolo_ms = elapsed_ms(yolo_start_ns)
                prediction_mailbox.mark_yolo(utime.ticks_ms())

                algorithm_start_ns = time.time_ns()
                center_x = select_ball_center(det_boxes)
                if center_x is None:
                    if position_filter.handle_empty_frame() is None:
                        velocity_tracker.reset()
                        prediction_mailbox.reset_tracking()
                else:
                    filtered_center_x = position_filter.update(center_x)
                    velocity = velocity_tracker.update(filtered_center_x,
                                                       utime.ticks_ms())
                    if velocity is None:
                        prediction_mailbox.reset_tracking()
                    else:
                        prediction_mailbox.accept_measurement(
                            center_x, velocity_tracker.last_signed_velocity,
                            utime.ticks_ms())
                algorithm_ms = elapsed_ms(algorithm_start_ns)

            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, last_draw_ms) >= DRAW_PERIOD_MS:
                draw_start_ns = time.time_ns()
                draw_detection_overlay(osd_img, det_boxes, labels,
                                       prediction_mailbox.overlay_status())
                draw_ms = elapsed_ms(draw_start_ns)
                last_draw_ms = now_ms

            rgb888p_img = None
            gc.collect()
            report_frame_timing(timing_profiler, capture_ms, yolo_ms, draw_ms,
                                algorithm_ms, frame_start_ns)
            next_yolo_ms = utime.ticks_add(next_yolo_ms, YOLO_PERIOD_MS)
            if utime.ticks_diff(utime.ticks_ms(), next_yolo_ms) > 0:
                next_yolo_ms = utime.ticks_ms()

    except KeyboardInterrupt:
        print("stopped")
    except Exception as error:
        print("An error occurred during detection: %s" % error)
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        if output_worker is not None:
            output_worker.stop()
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        if uart is not None:
            uart.deinit()
        del ai2d_input_tensor
        del ai2d_output_tensor
        gc.collect()
        nn.shrink_memory_pool()

    print("det_infer end")


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    wait_for_ide_interrupt()
    detection()
