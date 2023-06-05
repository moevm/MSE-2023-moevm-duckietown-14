import numpy as np
import imutils
import cv2

class MorphologicalLaneDetector:
    def __init__(self, roi, horizontal_kernel_size=10, vertical_kernel_size=100):
        self.roi = roi
        self.hks = horizontal_kernel_size
        self.vks = vertical_kernel_size
        self.LDPoint = None
        self.RDPoint = None

    
    def ChangeROI(self, newROI): # I am not sure if we need this function
        self.roi = newROI
    
    def applyROIMask(self, frame, save=False, plot=False):
        mask = np.zeros(frame.shape[:2], np.uint8)
        mask = cv2.fillConvexPoly(mask, self.roi.region , 255)
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

        if save:
            cv2.imwrite("masked_frame.jpg", masked_frame)
        if plot:
            cv2.imshow("", masked_frame)

        return masked_frame

    def StopLineSearch(self, frame, save=False, plot=False):
        # Specify size on horizontal axis
        cols = frame.shape[1]
        horizontal_size = cols // self.hks
        # Create structure element for extracting horizontal lines through morphology operations
        horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_size, 1))
        # Apply morphology operations
        horizontal = cv2.erode(frame, horizontalStructure)
        horizontal = cv2.dilate(horizontal, horizontalStructure)

        if save:
            cv2.imwrite("stop_line.jpg", horizontal)
        if plot:
            cv2.imshow("",horizontal)
        
        return horizontal

    def LaneLinesSearch(self, frame, save=False, plot=False):
        # Specify size on vertical axis
        rows = frame.shape[0]
        verticalsize = rows // self.vks
        # Create structure element for extracting vertical lines through morphology operations
        verticalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, verticalsize))
        # Apply morphology operations
        vertical = cv2.erode(frame, verticalStructure)
        vertical = cv2.dilate(vertical, verticalStructure)

        if save:
            cv2.imwrite("stop_line.jpg", vertical)
        if plot:
            cv2.imshow("",vertical)

        return vertical

    def GrabRedLineCorners(self, frame, orig_frame, save=False, plot=False):
        cnts = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        contour = max(cnts, key=cv2.contourArea)
        RDPoint = sorted(contour, key=lambda x:sum(x[0]), reverse=True)[0][0]
        LDPoint = sorted(contour, key=lambda x:x[0][0])[0][0]
        RDPoint[1] = LDPoint[1]
        self.LDPoint, self.RDPoint = LDPoint, RDPoint

        if plot:
            frame_hc = cv2.circle(orig_frame, LDPoint, 5, (255, 0, 255), -1)
            frame_hc = cv2.circle(frame_hc, RDPoint, 5, (255, 0, 255), -1)
            cv2.imshow("",frame_hc)
            if save:
                cv2.imwrite("highlited_corners.jpg", frame_hc)
        return

    def SamplePoints(self, contours, sample_size=4):
        
        sample = []
        if len(contours) == 0:
            return sample
        # deterministic part
        for cnt in contours:
            tgt = max(cnt, key=cv2.contourArea)
            cnt = np.setdiff1d(tgt, cnt)
            sample.append(tgt[0])
        # ramdomized part to make better predictions
        if len(sample) < sample_size:

            indexX = [contours[np.random.randint(len(contours))] for _ in range(sample_size - len(sample))]
            #indexX = np.random.choi(contours, sample_size - len(sample), replace=True)
            for index in indexX:
                sampled = index[np.random.randint(len(index))]
                index = np.setdiff1d(index, sampled)
                sample.append(sampled[0])
        return sample


    def splitScreenPoints(self, thresh, contours):
        left_contours = []
        right_contours = []
        for cnt in contours:
            xmin = min(cnt, key=lambda x:x[0][0])[0][0]
            if  xmin < thresh:
                left_contours.append(cnt)
            else:
                right_contours.append(cnt)
        left_points, right_points = self.SamplePoints(left_contours), self.SamplePoints(right_contours)
        return left_points, right_points        

    def HihglightRegion(self, frame, orig_frame=None, ignore_stop_line=False, save=False, plot=False):
        # Let's find the contours of the lanes
        cnts = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # Contours are great, but we need points to fit the lines through
        #LDPs = []
        #for cnt in cnts:
        #   LDPs.append(max(cnt, key=cv2.contourArea)[0])

        # Next step is to separate the points
        thresh = frame.shape[1]/2 + 60
        left_points = []
        right_points = []
        
        left_points, right_points = self.splitScreenPoints(thresh, contours=cnts)
        # Enhanced Sampling
        if not ignore_stop_line:
            left_points.append(self.LDPoint)
            right_points.append(self.RDPoint)
        # Next step is to separate the coordinates
        lefty, leftx = [], []
        for x, y in left_points:
            leftx.append(x)
            lefty.append(y)
        righty, rightx = [], []
        for x, y in right_points:
            rightx.append(x)
            righty.append(y)
        # Finally fit points
        left_fit = np.polyfit(leftx, lefty, 1)
        right_fit = np.polyfit(rightx, righty, 1)
        # Make polynomials from Least Squares coefficients
        pl = np.poly1d(left_fit)
        pr = np.poly1d(right_fit)
        # Save the polynomials
        self.right_polynom = pr
        self.left_polynom = pl
        
        if plot:
            # plot points
            for point in right_points:
                orig_frame = cv2.circle(orig_frame, point, 5, (255, 0, 0), -1)
            for point in left_points:
                orig_frame = cv2.circle(orig_frame, point, 5, (255, 255, 0), -1)
            if not ignore_stop_line:
                orig_frame = cv2.circle(orig_frame, self.LDPoint, 5, (255, 0, 255), -1)
                orig_frame = cv2.circle(orig_frame, self.RDPoint, 5, (255, 0, 255), -1)
            cv2.imshow("",orig_frame)

            # plot line
            if not ignore_stop_line:
                for x in range(orig_frame.shape[1]):
                    if pl(x) < orig_frame.shape[0] and x < self.LDPoint[0]:
                        orig_frame[int(pl(x))][x] = [255, 0, 255]
                    if 0 < pr(x) < orig_frame.shape[0] and x > self.RDPoint[0]:
                        orig_frame[int(pr(x))][x] = [255, 0, 255]
            else:
                for x in range(orig_frame.shape[1]):
                    if pl(x) < orig_frame.shape[0] and x > self.LDPoint[0]:
                        orig_frame[int(pl(x))][x] = [255, 0, 255]
                    if 0 < pr(x) < orig_frame.shape[0] and x < self.RDPoint[0]:
                        orig_frame[int(pr(x))][x] = [255, 0, 255]
            cv2.imshow("",orig_frame)
            
            #plot green area
            if type(orig_frame) != type(None):
                if not ignore_stop_line:
                    for y in range(orig_frame.shape[0]):
                        for x in range(orig_frame.shape[1]):
                            if y > self.LDPoint[1] and  pr(x) < y and pl(x) < y:
                                orig_frame[y][x][1] = 150
                else:
                    for y in range(orig_frame.shape[0]):
                        for x in range(orig_frame.shape[1]):
                            if  pr(x) < y and pl(x) < y:
                                orig_frame[y][x][1] = 150
                if save:
                    #cv2.imwrite("image_out.jpg", orig_frame)
                    return orig_frame
            cv2.imshow("",orig_frame)

        

    def FilterOutStopLine(self, masked_frame, stop_line_frame, save=False, plot=False):
        masked_frame_minus = cv2.bitwise_and(stop_line_frame, masked_frame)
        masked_frame_minus = cv2.bitwise_xor(masked_frame_minus, masked_frame)
        if save:
            cv2.imwrite("only_vertical.jpg", masked_frame_minus)
        if plot:
            cv2.imshow("",masked_frame_minus)
        return masked_frame_minus