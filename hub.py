import airsim
import numpy as np
import cv2


class Hub:
    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def get_cur_state(self):
        return self.client.getCarState()

    def get_scene_view_4ch(self):
        response = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        return img1d.reshape(response.height, response.width, 4)

    def get_scene_view_3ch(self):
        return cv2.cvtColor(self.get_scene_view_4ch(), cv2.COLOR_BGRA2RGB)

    def get_depth_perspective_4ch(self):
        response = self.client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.DepthVis, False, False)])[0]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        return img1d.reshape(response.height, response.width, 4)

    def get_depth_perspective_3ch(self):
        return cv2.cvtColor(self.get_depth_perspective_4ch(), cv2.COLOR_BGRA2RGB)

    def enable_api(self, val):
        self.client.enableApiControl(val)

    def set_throttle(self, value):
        self.car_controls.handbrake = False
        if value > 0:
            self.car_controls.throttle = value
            self.car_controls.brake = 0
        elif value < 0:
            self.car_controls.throttle = 0
            self.car_controls.brake = value
        elif value == 0:
            self.car_controls.throttle = 0
            self.car_controls.brake = 1
            #print("Brakes: " + str(self.car_controls.brake))

    def steer(self, turn):
        self.car_controls.steering = turn
        #print("Turn: " + str(turn))

    def apply_controls(self):
        self.client.setCarControls(self.car_controls)
