from LaneDetector.ROI import *
from LaneDetector.GradientImagePreprocessor import *
from LaneDetector.MorphologicalLaneDetector import *

class IntersectionController:
    # resolution (height, width)
    def __init__(self, edge_kernel, resolution=(480, 640)):
        #self.RROI =
        #self.LROI = 
        self.ROI = ROI()
        self.res = resolution
        self.GIP = GradientImagePreprocessor(edge_kernel)
        self.MLD = MorphologicalLaneDetector(self.ROI)

    def renderImage(self, frame):
        # 1st Node [Find Edges] frame -> frame
        edges = self.GIP.get_line_markings(frame.copy(), plot=False, save=False)
        # 2nd Node [Apply Mask] frame -> frame
        masked_frame = self.MLD.applyROIMask(edges.copy(), plot=True, save=False)
        # 3rd Node [Find Stopline] frame -> frame
        stop_line = self.MLD.StopLineSearch(masked_frame.copy(), plot=True, save=False)
        # 4th Node [grab corners] frame -> None
        self.MLD.GrabRedLineCorners(stop_line, orig_frame=frame.copy(), save=False, plot=False)
        # 5th [Shrink the ROI] Two points -> None
        self.ROI.shrink(self.MLD.LDPoint, self.MLD.RDPoint)
        # 6th [Apply ROI mask] frame -> frame
        masked_frame = self.MLD.applyROIMask(edges.copy(), plot=False, save=False)
        # 7th [Erase stop line to detect Lane Lines] frame -> frame
        filtered_frame = self.MLD.FilterOutStopLine(masked_frame, stop_line, plot=True, save=False)
        # 8th [Search for the lane lines] frame -> frame
        lane_lines = self.MLD.LaneLinesSearch(filtered_frame.copy(), plot=False, save=False)
        # 9th [Region highlihting; will be replaced by line drawing] frame -> None (will return frame)
        return self.MLD.HihglightRegion(lane_lines, orig_frame=frame.copy(), save=True, plot=True)


    def ROIupdate(self, new_ROI=None):
        if type(new_ROI) != type(None):
            self.ROI = new_ROI
            return

    def ChangeDirection(self, roi, direction, new_bl, new_br):
        if direction == "right":
            roi.update(new_region=np.int32([(530, 218), tuple(new_bl), tuple(new_br), (640,350)]))
        elif direction == "left":
            roi.update(new_region=np.int32([(110, 218), tuple(new_bl), tuple(new_br), (0,350)]))
        elif direction == "forward":
            roi.update(new_region=np.int32([(330, 190), tuple(new_bl), tuple(new_br), (430, 190)]))
        return roi.region

    def GetSquare(self):
        tl = self.ROI[0]
        tr = self.ROI[3]
        roiSquare =  (tr[0] - tl[0]) * (self.res[0] - tl[1])
        return roiSquare

