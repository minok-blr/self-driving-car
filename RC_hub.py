import serial
from hub import Hub
import time
import cv2
from skin_detection import SkinDetection
from opt_flow import OptFlow
from autodriver import AutoDriver
import numpy as np
from datetime import datetime
import math

def printLog():
    print("INCOMING DATA:")
    print("Roll:" + roll)
    print("Pitch:" + pitch)
    print("Yaw:" + yaw)
    print("Thrust:" + str(thrust))
    print("Steer:" + str(steer))
    print("END\n")

hub = Hub()
#prev_turn = 0
#turn_accumulator = 0
overflow = 0

# AI CONTROL VARS
timeout_ms = 25
#hub = Hub()
skin_detector = SkinDetection()
autodriver = AutoDriver(0.37)
img = cv2.cvtColor(hub.get_scene_view_3ch(), cv2.COLOR_RGB2HSV)
turn_detector = OptFlow(img)
prev_time = datetime.now()
AI_time_set_once = 0

prev_turn = 0
turn_accumulator = 0
prev_dist = 0
dist_accumulator = 0
prev_speed = 0
speed_accumulator = 0
use_ai = 0

# OPEN USB PORT
ser = serial.Serial('COM3', 115200)
ser.flushInput()
ser.flushOutput()

while True:
    #time.sleep(0.1)
    data_raw = ser.readline()
    print(data_raw)
    data_decoded = data_raw.decode().strip("\r")
    #print(data_decoded)
    #print(data_decoded.partition(":")[0])
    
    if data_decoded.partition(":")[0] == "AI_CTRL":
        use_ai = 1
        if AI_time_set_once == 0:
            prev_time = datetime.now()
            AI_time_set_once = 1

        #print("AI CONTROL")
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
                #print("Throttle: " + str(speed))
                hub.set_throttle(speed)

        hub.apply_controls()

        #print("Turn:", turn)
        #print("Dist:", dist_diff)

        
    elif data_decoded.split()[0].partition(":")[0][2:-1]:
        if overflow == 25:
            #printLog()
            overflow = 0
        else:
            overflow = overflow + 1
        AI_time_set_once = 0
        use_ai = 0

        data_raw = data_raw.split()
        #print(data_raw)
        #roll
        roll = str(data_raw[1]).partition(":")
        roll = roll[2][:-2]
        #pitch
        pitch = str(data_raw[2]).partition(":")
        pitch = pitch[2][:-2]
        #yaw
        yaw = str(data_raw[3]).partition(":")
        yaw = yaw[2][:-2]
        
        max_value_pitch = 83.0 # should be 90
        max_value_roll = 90.0 

        if float(pitch) > 83.0: #should be 90
            pitch == 83.0
        
        # normalise between -1 and 1
        thrust = float(pitch) / max_value_pitch
        steer = float(roll) / max_value_roll
        
        if not isinstance(thrust, type(None)):
            hub.set_throttle(-thrust)
            hub.steer(steer)

        hub.apply_controls()