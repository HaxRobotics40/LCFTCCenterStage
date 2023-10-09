package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.ApriltagDetectionJNI;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SleeveDetector extends OpenCvPipeline {

    private long nativeApriltagPtr;
    private long alldetectionsptr;
    long[] detectionptr1;
    private int output;

    Mat cameraMatrix;
    private IndicatedLocation location = IndicatedLocation.START;

    Mat frame = new Mat();

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;
    double tagsizeX;
    double tagsizeY;

    Telemetry telemetry;


    public SleeveDetector (Telemetry telemetry)
    {
        tagsize = 0.0325755;
        tagsizeX = 0.0325755;
        tagsizeY = 0.0325755;
        fx = 162.051861606;
        fy = 162.051861606;
        cx = 360;
        cy = 360;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        this.telemetry = telemetry;
    }

    // public void finalize()
    // {
    //     // Might be null if createApriltagDetector() threw an exception
    //     if(nativeApriltagPtr != 0)
    //     {
    //         // Delete the native context we created in the constructor
    //         return;
    //     }
    //     else
    //     {
    //         AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
    //         location = IndicatedLocation.NONE;
    //         nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    //     }
    // }
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, frame, Imgproc.COLOR_BGR2GRAY);

        if (frame.empty()) {
            location = IndicatedLocation.NONE;
            return input;
        }

        //make a detector for 36h11, then use the detector on the Mat called frame
        // then use the info obtained from that using Detection instead of Detector
        // and get the ID - and then if statement for if the ID is this, then return this location
        // it SHOULD work with the pipeline because it's using processFrame
//        All the detection methods take the input of a long(pointer) to an array, but simple returns the array itself


        alldetectionsptr = AprilTagDetectorJNI.runApriltagDetector(nativeApriltagPtr, frame.dataAddr(), frame.width(), frame.height());


        // this is trying to look for null and if it is null it random's a detection - try with solely information of id and then try real
        if (alldetectionsptr != 0) {
            // the above is the issue because running getdetectionpotienrs on a null pointer doesnt work and thats the erroring part
            detectionptr1 = ApriltagDetectionJNI.getDetectionPointers(alldetectionsptr);

            output = ApriltagDetectionJNI.getId(detectionptr1[0]);

            ApriltagDetectionJNI.freeDetectionList(alldetectionsptr);

        } else {

            double thing = Math.random();
            long thing1 = Math.round(thing*3);

            if (thing1 == 1) {
                output = 40;
            } else if (thing1 == 2) {
                output = 125;
            } else {
                output = 400;
            }
            // detectionptr1 = new long[1];
            // detectionptr1[0] = 40404040;
        }

        // int[] detIDs = new int[detectionptr1.length];
        // for(int i = 0; i < detectionptr1.length; i++){
        //     detIDs[i] = ApriltagDetectionJNI.getId(detectionptr1[i]);
        // }
        for (long detection : detectionptr1) {
            telemetry.addData("output: ", Integer.toString(ApriltagDetectionJNI.getId(detection)));
        }
        telemetry.update();


        if (output == 40) {
            location = IndicatedLocation.LEFT;
            return frame;
        } else if (output == 125) {
            location = IndicatedLocation.MIDDLE;
            return frame;
        } else if (output == 400) {
            location = IndicatedLocation.RIGHT;
            return frame;
        } else {
            location = IndicatedLocation.NOTHING;
            return frame;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    public IndicatedLocation getLocation() {
        return this.location;
    } // was getLocation

    public int getLocationInt() {
        return location.val;
    } // lol fix this

    public long getDetectionPtrLong() {
        return detectionptr1[0];
    }

    public long getAlldetectionsptr() {
        return alldetectionsptr;
    }

    public enum IndicatedLocation {
        LEFT(1),
        MIDDLE(2),
        RIGHT(3),
        NONE(0),
        START(4),
        NOTHING(5);

        int val;


        IndicatedLocation(int i) {
            val = i;
        }

        public int val() {
            return val;
        }
    }


}


