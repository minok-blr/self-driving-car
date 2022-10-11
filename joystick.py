import serial
from hub import Hub

def printLog():
    print("Roll:" + roll)
    print("Pitch:" + pitch)
    print("Yaw:" + yaw)
    print("Thrust:" + str(thrust))
    print("Steer:" + str(steer))

hub = Hub()
prev_turn = 0
turn_accumulator = 0

# OPEN USB PORT
ser = serial.Serial('COM3', 96000)
ser.flushInput()
ser.flushOutput()

overflow = 0
while True:
    if overflow == 50:
        printLog()
        overflow = 0
    else:
        overflow = overflow + 1

    data_raw = ser.readline()
    data_raw = data_raw.split()

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