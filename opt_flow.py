import cv2
import numpy as np


class OptFlow:
    def __init__(self, img):
        # Parameters for Shi-Tomasi corner detection
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        try:
            self.prev_features = cv2.goodFeaturesToTrack(img, mask=None, **self.feature_params)
        except:
            self.prev_features = []
        self.prev_img = img

    def __call__(self, img):
        img_center = img.shape[0] / 2
        mean_dist = 0

        if not isinstance(self.prev_features, type(None)) and len(self.prev_features) > 0:
            try:
                features, status, error = cv2.calcOpticalFlowPyrLK(self.prev_img, img, self.prev_features,
                                                                   None, **self.lk_params)
                good = features[status == 1]
                distances = []
                print("Center:", img_center)
                for i, (x, y) in enumerate(good):
                    print("X:", x)
                    distances.append(x - img_center)
                # get the mean distance between detected features from the second image and its center in degrees
                mean_dist = (np.array(distances).mean() * 45) / img_center
            finally:
                pass
        else:
            try:
                features = cv2.goodFeaturesToTrack(img, mask=None, **self.feature_params)
            finally:
                pass

        if len(features) > 0:
            self.prev_img = img
            self.prev_features = features
        return mean_dist
