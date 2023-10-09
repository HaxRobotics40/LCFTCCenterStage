// package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class ShippingElementDetector extends OpenCvPipeline {
   private final int width = 900;
   private ElementLocation location = ElementLocation.NONE;
   Mat mat = new Mat();
   Telemetry telemetry;
   public ShippingElementDetector(Telemetry telemetry) {
    this.telemetry = telemetry;
}

   @Override
   public Mat processFrame(Mat input) {
       // Make a working copy of the input matrix in HSV
       Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

       // if there isn't anything detected, assume something is wrong
       if (mat.empty()) {
           location = ElementLocation.NONE;
           return input;
       }

       // create an HSV range for neon green to detect shipping element
       Scalar lowHSV = new Scalar(50, 100, 100); // lower bound for HSV range
       Scalar highHSV = new Scalar(60, 255, 255); // higher bound for HSV range
       Mat thresh = new Mat();

       // returns a black and white image. The white regions represent the shipping element
       // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
       Core.inRange(mat, lowHSV, highHSV, thresh);

       // edge detection
       Mat edges = new Mat();
       Imgproc.Canny(thresh, edges, 100, 300);
       thresh.release();
       // oftentimes the edges are disconnected, findContours connects these edges
       // then find the bounding rectangles of those contours
       List<MatOfPoint> contours = new ArrayList<>();
       Mat hierarchy = new Mat();
       Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
       hierarchy.release();
       edges.release();
       MatOfPoint2f contoursPoly;
       Rect[] boundRect = new Rect[contours.size()];
       for (int i = 0; i < contours.size(); i++) {
           contoursPoly = new MatOfPoint2f();
           Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly, 3, true);
           contours.get(i).release();
           boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly.toArray()));
           contoursPoly.release();
       }

       // iterate and check whether the bounding boxes cover left and/or right side of the image
       double mid_x = 0.5 * width;
       boolean left = false; // true if regular stone found on the left side
       boolean middle = false; // "" "" on the right side
       for (int i = 0; i != boundRect.length; i++) {
           if (boundRect[i].x < mid_x) {
               left = true;
           }
           if (boundRect[i].x + boundRect[i].width > mid_x && boundRect[i].x < 0.8 * width) {
               middle = true;
           }
           // draw red bounding rectangles on mat, the mat has been converted to HSV
           // so we need to use HSV as well
           Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
       }

       // if there is neon green on a side, that side should be the element
       // if both are false or both are true, then element must be in the middle
       if (left && !middle) {
           location = ElementLocation.LEFT;
       } else if (middle && !left) {
           location = ElementLocation.MIDDLE;
       } else {
           location = ElementLocation.RIGHT;
       }

       telemetry.addData("Location: ", location);
       telemetry.update();

       return mat; // return the mat with rectangles drawn
   }

   public ElementLocation getLocation() {
       return this.location;
   }

   public int getLocationInt() {
       return location.val;
   }

   public enum ElementLocation {
       LEFT(1),
       MIDDLE(2),
       RIGHT(3),
       NONE(0);

       int val;


       ElementLocation(int i) {
           val = i;
       }

       public int val() {
           return val;
       }
   }
}