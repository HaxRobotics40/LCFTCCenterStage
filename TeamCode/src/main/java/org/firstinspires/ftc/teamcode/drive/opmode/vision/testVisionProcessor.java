package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;
import android.graphics.Rect;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;

public class testVisionProcessor implements VisionProcessor {
    // TODO: make this a vision processor later, it's the exact same thing (see: LJ4FTC)
    Mat secondary = new Mat();
    Mat edges = new Mat();
    int location;
    Rect rect = new Rect(20, 20, 50, 50);
    final int width = 900; // TODO: add as constructor parameter when done testing
    int color;

    public void init(int width, int height, CameraCalibration calibration) {

    }
    Telemetry telemetry; //TODO: get rid of all instances of Telemtry as an object
    public testVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public Mat processFrame(Mat input, long captureTimeNanos) {

        Imgproc.cvtColor(input, secondary, Imgproc.COLOR_RGB2HSV);

        if (secondary.empty()) {
            location = -1;
            return secondary;
        }

        // ArrayList<Scalar> list = new ArrayList<>();



        Scalar[] lower_RB = {
                new Scalar(0, 90, 70),
                new Scalar(70, 70, 50),
                new Scalar(160,140,145)
        };
        Scalar[] upper_RB = {
                new Scalar(7, 205, 225),
                new Scalar(150, 220, 170),
                new Scalar(190,190,205)
        };



        Mat maskR = new Mat();
        Mat maskB = new Mat();
        Mat maskR2 = new Mat();
        Mat colorOnly = new Mat();
        Mat mask = new Mat();
        int amountColor;
        int type = colorOnly.type();

        Core.inRange(secondary, lower_RB[0], upper_RB[0], maskR);
        Core.inRange(secondary, lower_RB[1], upper_RB[1], maskB);
        Core.inRange(secondary, lower_RB[2], upper_RB[2], maskR2);

        Core.bitwise_and(input, input, colorOnly, maskR = maskR);
        Core.bitwise_and(input, input, colorOnly, maskB = maskB);
        Core.bitwise_and(input, input, colorOnly, maskR2 = maskR2);
        Core.bitwise_and(maskR, maskR, mask, maskR = maskR);
        Core.bitwise_and(maskB, maskB, mask, maskB = maskB);
        Core.bitwise_and(maskR2, maskR2, mask, maskR2 = maskR2);
        secondary.release();
        maskR.release();
        maskB.release();
        maskR2.release();
        colorOnly.release();


        // Imgproc.Canny(mask, edges, 100, 300);
        Photo.fastNlMeansDenoising(mask, edges, 1, 7, 21); // fix h, keeps crashing

        // edges = mask.clone();

        // Imgproc.cvtColor(mask, edges, Imgproc.COLOR_RGB2GRAY);
        // figure out how to see where the largest boxes are
        // List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        // Mat hierarchy = new Mat();
        // Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // for (int i=0; i<contours.size(); i++) {
        //     //Convert contours(i) from MatOfPoint to MatOfPoint2f
        //     MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
        //     //Processing on mMOP2f1 which is in type MatOfPoint2f
        //     double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
        //     Imgproc.approxPolyDP(contour2f, contour2f, approxDistance, true);

        //     //Convert back to MatOfPoint
        //     MatOfPoint points = new MatOfPoint( contour2f.toArray() );

        //     // Get bounding rect of contour
        //     rect = Imgproc.boundingRect(points);

        //     // draw enclosing rectangle (all same color, but you could use variable i to make them unique)
        //     Imgproc.rectangle(edges, new Point(rect.x,rect.y), new Point(rect.x+rect.width,rect.y+rect.height), new Scalar(225,0,225));

        // }

        // comment this out when implementing on the bot
        telemetry.addData("Location", location);
        telemetry.update();


        return edges;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        canvas.drawRect(rect, selectedPaint);
    }

    public int pieceLocation() {
        return location;
    }
}
