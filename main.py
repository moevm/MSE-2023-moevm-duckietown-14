import cv2
from LaneDetector.IntersectionController import *
from LaneDetector.EdgeKernels import *


frame = cv2.imread("./test_image.png")
IC = IntersectionController(scharr)
IC.renderImage(frame)