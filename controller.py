import cv2
from hub import Hub
from skin_detection import SkinDetection
from opt_flow import OptFlow
from autodriver import AutoDriver
import numpy as np
from datetime import datetime
import math

timeout_ms = 25
hub = Hub()
skin_detector = SkinDetection()
autodriver = AutoDriver(0.37)
img = cv2.cvtColor(hub.get_scene_view_3ch(), cv2.COLOR_RGB2HSV)
turn_detector = OptFlow(img)
prev_time = datetime.now()

prev_turn = 0
turn_accumulator = 0
prev_dist = 0
dist_accumulator = 0
prev_speed = 0
speed_accumulator = 0

while True:
    img_bgr = hub.get_scene_view_3ch()
    mask = skin_detector(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV))
    indices = np.nonzero(mask > 0)
    turn = (np.mean(indices[1]) - (mask.shape[1] / 2)) * 2 / mask.shape[1]
    dist = np.mean(indices[0]) / mask.shape[0]

    if turn == 0.0:
        turn_accumulator = turn_accumulator + 1

    if dist == 0.0:
        dist_accumulator = dist_accumulator + 1

    if not math.isnan(turn):
        prev_turn = turn

    if not math.isnan(dist):
        prev_dist = dist

    if isinstance(turn, np.float64):
        if math.isnan(turn) or turn_accumulator == 50:
            turn = prev_turn
            turn_accumulator = 0
        hub.steer(turn * 2)

    if isinstance(dist, np.float64):
        if math.isnan(dist) or dist_accumulator == 50:
            dist = prev_dist
            dist_accumulator = 0

    if not math.isnan(dist):
        now = datetime.now()
        dist_diff = (1 - dist)
        dt = (now - prev_time).total_seconds()
        autodriver.updateMeasurement(dt, dist_diff)
        speed = autodriver.thrust / 3
           
        prev_time = now
        if not math.isnan(speed):
            prev_speed = speed
        if math.isnan(speed):
            speed = prev_speed

        # print(speed, dist_diff, dt)
        if not isinstance(speed, type(None)):
            print("Throttle: " + str(speed))
            hub.set_throttle(speed)

    hub.apply_controls()

    print("Turn:", turn)
    print("Dist:", dist_diff)