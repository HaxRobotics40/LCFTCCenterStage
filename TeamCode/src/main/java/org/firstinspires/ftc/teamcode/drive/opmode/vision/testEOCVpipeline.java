package org.firstinspires.ftc.teamcode.drive.opmode.vision;

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

/*
steps for EOCV sim
1. create 2 folders under users/(name)/, one for images(if applicable) and one for code
2. copy paste everything in vision to the code folder
3. run eocv sim, Workspace -> Select workspace, select code folder
4. (if images) Create -> Image -> Select images from folder, resize

edit the code in local folder then copy paste the entire folder back here when done

 */
public class testEOCVpipeline extends OpenCvPipeline {

    Mat secondary = new Mat();
    int location;
    private final int width = 900; // add to constructor as parameter when on bot

    // comment this out when implementing on the bot
    Telemetry telemetry;
    public testEOCVpipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public Mat processFrame(Mat input) {
        
        Imgproc.cvtColor(input, secondary, Imgproc.COLOR_RGB2HSV);

        if (secondary.empty()) {
            location = -1;
            return secondary;
        }
        
        // ArrayList<Scalar> list = new ArrayList<>();



        Scalar[] lower_RB = {
            new Scalar(0, 100, 130), 
            new Scalar(70, 50, 50)
        };
        Scalar[] upper_RB = {
            new Scalar(10, 255, 225),
            new Scalar(150, 220, 170)
        };



        Mat mask = new Mat();
        Mat colorOnly = new Mat();
        int amountColor;

        for (int i=0; i<2; i++) {
            Core.inRange(secondary, lower_RB[i], upper_RB[i], mask);
            Core.bitwise_and(input, input, colorOnly, mask = mask);

            amountColor = Core.countNonZero(colorOnly);
            if (amountColor == 0) {
                continue;
            } else {
                break;
            }
        }



        // comment this out when implementing on the bot
        telemetry.addData("Location", location);
        telemetry.update();

        return colorOnly;
    }

    public int getPieceLocation() {
        return location;
    }
}
