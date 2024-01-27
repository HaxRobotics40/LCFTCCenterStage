package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;
import org.openftc.easyopencv.OpenCvPipeline;
import android.graphics.Canvas;

import java.util.ArrayList;
import java.util.List;

public class testEOCVpipeline implements VisionProcessor {
    Mat secondary = new Mat();
    int location = -1;
    private String color;


    public void init(int width, int height, CameraCalibration cameraCalibration) {

    }
    public Mat processFrame(Mat input, long e) {

        Imgproc.cvtColor(input, secondary, Imgproc.COLOR_RGB2HSV);

        if (secondary.empty()) {
            return secondary;
        }

        // ArrayList<Scalar> list = new ArrayList<>();



        Scalar[] lower_RB = {
                new Scalar(0, 90, 70),
                new Scalar(70, 70, 50),
                new Scalar(170,20,110)
        };
        Scalar[] upper_RB = {
                new Scalar(7, 205, 225),
                new Scalar(150, 220, 220),
                new Scalar(180,130,180)
        };



        Mat maskR = new Mat();
        Mat maskB = new Mat();
        Mat maskR2 = new Mat();
//        Mat colorOnly = new Mat();
        Mat mask = new Mat();
        Mat edges = new Mat();
//        int amountColor;
//        int type = colorOnly.type();

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));

        Core.inRange(secondary, lower_RB[0], upper_RB[0], maskR);
        Core.inRange(secondary, lower_RB[1], upper_RB[1], maskB);
        Core.inRange(secondary, lower_RB[2], upper_RB[2], maskR2);

        // Perform opening (erosion followed by dilation) to remove noise on mask
        Imgproc.morphologyEx(maskR, maskR, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(maskB, maskB, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(maskR2, maskR2, Imgproc.MORPH_OPEN, kernel);

        // Perform closing (dilation followed by erosion) to fill small holes if required
        Imgproc.morphologyEx(maskR, maskR, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(maskB, maskB, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(maskR2, maskR2, Imgproc.MORPH_CLOSE, kernel);

//        Core.bitwise_and(input, input, colorOnly, maskR = maskR);
//        Core.bitwise_and(input, input, colorOnly, maskB = maskB);
//        Core.bitwise_and(input, input, colorOnly, maskR2 = maskR2);
        if (Core.countNonZero(maskR) + Core.countNonZero(maskR2) > Core.countNonZero(maskB)) {
            color = "RED";
        } else {
            color = "BLUE";
        }
        Core.bitwise_and(maskR, maskR, mask, maskR = maskR);
        Core.bitwise_and(maskB, maskB, mask, maskB = maskB);
        Core.bitwise_and(maskR2, maskR2, mask, maskR2 = maskR2);
        maskR.release();
        maskB.release();
        maskR2.release();
//        colorOnly.release();

        int width = mask.cols(); // Get the width of the mask
        int sectorWidth = width / 3; // Divide it by 3 to get each sector's width

// Define Regions of Interest (ROI) for each sector
        Rect leftSector = new Rect(0, 0, sectorWidth, mask.rows());
        Rect middleSector = new Rect(sectorWidth, 0, sectorWidth, mask.rows());
        Rect rightSector = new Rect(2 * sectorWidth, 0, sectorWidth, mask.rows());

// Count number of 1s in each sector
        int leftCount = Core.countNonZero(new Mat(mask, leftSector));
        int middleCount = Core.countNonZero(new Mat(mask, middleSector));
        int rightCount = Core.countNonZero(new Mat(mask, rightSector));

// Determine which sector has the most number of 1s
        int maxCount = Math.max(leftCount, Math.max(middleCount, rightCount));
//        String sectorWithMostOnes;

        if(maxCount == leftCount) {
            location = 0;
        } else if(maxCount == middleCount) {
            location = 1;
        } else {
            location = 2;
        }







        // Imgproc.Canny(mask, edges, 100, 300);
//        Photo.fastNlMeansDenoising(mask, edges, 1, 7, 21); // fix h, keeps crashing

         edges = mask.clone();

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i=0; i<contours.size(); i++) {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
            Imgproc.approxPolyDP(contour2f, contour2f, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint( contour2f.toArray() );

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            // draw enclosing rectangle (all same color, but you could use variable i to make them unique)
            Imgproc.rectangle(edges, new Point(rect.x,rect.y), new Point(rect.x+rect.width,rect.y+rect.height), new Scalar(225,0,225));

        }

        double midline = 0.5 * width;

        return edges;
    }
    public String getColor() { return color; }
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    public int locationInt() {
        return location;
    }
}
