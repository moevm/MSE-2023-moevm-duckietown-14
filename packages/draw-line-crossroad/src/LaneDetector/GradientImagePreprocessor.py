import cv2
import numpy as np

class GradientImagePreprocessor():
    def __init__(self, operator):
        self.apply_operator = operator
        self.line_markings = None

    # FIX FREAKING BINARY THRESHOLDING IT'S A CRAPPY CODE

    def binary_thresholding(self, array, threshold, value=1):
        if value == 0:
            binary = np.ones_like(array) 
        else:
            binary = np.zeros_like(array)  
            value = 1
        binary[(array >= threshold[0]) & (array <= threshold[1])] = value
        return binary
    
    def get_line_markings(self, frame, save=False, plot=False):
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS) # (blue, green, red) -> (hue, light, saturation)
        light = hls[:, :, 1]
        saturation = hls[:, :, 2]
        red = frame[:, :, 2]
            
            # light thresholding / lane line edges detection
        _, binary_bright_light = cv2.threshold(light, 120, 255, cv2.THRESH_BINARY) # x < 120 -> 0, x >= 120 -> 255
        blured_light = cv2.GaussianBlur(binary_bright_light, (3, 3), 0)
        grad = self.apply_operator(blured_light, 3) # apply operator
        high_grad = self.binary_thresholding(grad, (110, 255)) # apply threshold for sobel derivatives
 
            # saturation thresholding / lane line detection
        # saturation is the purity of the hue color we assume that lane lines are pure (>80), [like yellow or white]
        _, saturation = cv2.threshold(saturation, 20, 255, cv2.THRESH_BINARY)

            # yellow/white detection ([255, 255, 255] -> pure white) ([0, 255, 255] -> pure yellow)
        # White in the regions with the richest red channel values (e.g. >120).
        _, red = cv2.threshold(red, 120, 255, cv2.THRESH_BINARY)
 
            # Lane lines should be pure in color and have high red channel values    
        rs_binary = cv2.bitwise_and(saturation, red)

            # Combine the possible lane lines with the possible lane line edges
        self.edges = cv2.bitwise_or(rs_binary, high_grad.astype(np.uint8))    
        
        if save:
            cv2.imwrite("detected_edges.jpg", self.edges)
        if plot:
            cv2_imshow(self.edges)
        return self.edges