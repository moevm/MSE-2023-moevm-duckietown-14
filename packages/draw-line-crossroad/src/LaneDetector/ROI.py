import numpy as np

class ROI:
    # Order is [Top-left, Bottom-left, Bottom-right, Top-right]
    def __init__(self, tl=(274,210), bl=(0, 480), br=(640,480), tr=(460, 210)):
        self.region = np.int32([tl, bl, br, tr])

    def projection(self, point_a, point_b, y):
        return int((y-point_a[1])*(point_b[0] - point_a[0])/(point_b[1] - point_a[1])) + point_a[0]

    def plot(self, frame, save=False):
        for point in self.region:
             frame = cv2.circle(frame, point, 5, (0, 255, 255), -1)

        cv2_imshow(frame)

        if save:
            cv2.imwrite("frame_with_ROI.jpg", frame)

    
    def update(self, tl=None, bl=None, br=None, tr=None, new_region=[]):
        if len(new_region) != 0:
            self.region = new_region
            return
        else:
            new_region = [tl, bl, br, tr]
            for i in range(4):
                if type(new_region[i]) != type(None):
                    self.region[i] = new_region[i]

    def shrink(self, LDPoint, RDPoint):
        new_tl = (self.projection(self.region[1], self.region[0], LDPoint[1]), LDPoint[1])
        new_tr = (self.projection(self.region[2], self.region[3], RDPoint[1]), RDPoint[1])
        self.update(tl=new_tl, tr=new_tr)
        return