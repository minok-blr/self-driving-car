import cv2
import numpy as np


class SkinDetection:
    def __init__(self):
        self.lower = np.array([0, 48, 80], dtype=np.uint8)
        self.upper = np.array([20, 255, 255], dtype=np.uint8)
        # self.lower1 = np.array([0, 51, 40], dtype=np.uint8)
        # self.upper1 = np.array([13, 153, 255], dtype=np.uint8)
        # self.lower2 = np.array([166, 51, 40], dtype=np.uint8)
        # self.upper2 = np.array([179, 153, 255], dtype=np.uint8)

    def __call__(self, img_hsv):
        return cv2.inRange(img_hsv, self.lower, self.upper)
        # return np.bitwise_or(cv2.inRange(img_hsv, self.lower1, self.upper1),
        #                      cv2.inRange(img_hsv, self.lower2, self.upper2))
