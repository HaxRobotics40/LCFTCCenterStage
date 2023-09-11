package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



import java.util.*;
import org.opencv.core.*;

/*



 */

    @TeleOp
    public class AutoVisionTeleTest extends LinearOpMode {

        private AprilTagProcessor aprilTag;
        private VisionPortal visionPortal;
        public int targetTag;

        // localization variables, initialize with tests later
        private double targetBearing = 0;
        private double targetRange = 12.25;
        private double bearingThreshold = 7.5;
        private double rangeThreshold = 0.5;


        private final Locale LC = new Locale("en", "US"); // i dislike blocks of yellow highlights

        @Override
        public void runOpMode() {

            initAprilTag();

            targetTag = (int) Math.ceil(3*Math.random());


            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addData("Randomized Target Tag: ", targetTag);
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    telemetryAprilTag();

                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // Save CPU resources; can resume streaming when needed.
                    if (gamepad1.dpad_down) {
                        visionPortal.stopStreaming();
                    } else if (gamepad1.dpad_up) {
                        visionPortal.resumeStreaming();
                    }

                    // Share the CPU.
                    sleep(20);
                }
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }   // end runOpMode()

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {

            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");
            CameraName Camera = ClassFactory.getInstance()
                    .getCameraManager().nameForSwitchableCamera(webcam);

            // Create the vision portal by using a builder. Adds camera + processor
            visionPortal = new VisionPortal.Builder()
                    .setCamera(Camera)
                    .addProcessor(aprilTag)
                    .build();

        }   // end method initAprilTag()

        /**
         * Add telemetry about AprilTag detections.
         */
        private void telemetryAprilTag() {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == targetTag) {
                    telemetry.addData("Target AprilTag Detected:", true);
                    double range = detection.ftcPose.range;
                    double elevation = detection.ftcPose.elevation;
                    double bearing = detection.ftcPose.bearing;

                    telemetry.addData("Bearing (deg)", bearing);
                    telemetry.addData("Range (in)",  range);

                    // auto mode, include approach code before here to be in range
                    // if bearing threshold is too high, halve




//                    telemetry.addLine(String.format(LC, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                    telemetry.addLine(String.format(LC, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format(LC, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    telemetry.addLine(String.format(LC, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

        }



        /*
         * Set the active camera according to input from the gamepad.
         */
    }

