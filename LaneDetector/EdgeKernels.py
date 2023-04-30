import cv2
import numpy as np

def scharr(img, kernel): # better 3x3 than sobel 3x3
    grad_x = cv2.Scharr(img, cv2.CV_64F, 1, 0)
    grad_y = cv2.Scharr(img, cv2.CV_64F, 0, 1)
    grad_magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)
    return grad_magnitude

def sobel(img, kernel): # baseline method with nxn kernel, n is odd
    grad_x = cv2.Sobel(img, cv2.CV_64F, 1, 0, kernel)
    grad_y = cv2.Sobel(img, cv2.CV_64F, 0, 1, kernel)
    grad_magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)
    return grad_magnitude

def laplacian(img, kernel):
    grad2 = cv2.Laplacian(img, cv2.CV_64F, ksize=kernel)
    return grad2