package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Autonomous
public class AutonomousMode extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;

    SampleMecanumDrive drive;
    int targetTag = 1;
    double detX; //goal 0, means looking straight at tag
    double detY; // goal ?? depends per robot
    double targetY = 12;
    String direction;
    String strafe;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagProcessor = initAprilTag();
        vPortal = initVisionPortal(aprilTagProcessor);

//        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
//                May be blocking!!!
                aprilTagTelemetry();
                telemetry.update();
                List<AprilTagDetection> aprilTagDetectionsList = aprilTagProcessor.getDetections();
            }


        }
    }

    private AprilTagProcessor initAprilTag() {
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());

        return aprilTagProcessorBuilder.build();
    }

    private VisionPortal initVisionPortal(AprilTagProcessor atp) {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.addProcessor(atp);

        return vPortalBuilder.build();
    }

    private void endTickTelemetry() {
        aprilTagTelemetry();
    }

    @SuppressLint("DefaultLocale")
    private void aprilTagTelemetry() {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


                        // targetTag isolated
                        if (detection.id % 3 == targetTag || detection.id % 3 == targetTag-3) {
                            detX = detection.ftcPose.x;
                            detY = detection.ftcPose.y;

                            // assumign using sample mecanum drive, RR
                            //assuming robot made a perfect 90 deg turn and is just to the right or left of tag
                            if(detX != 0) {
                                if (detX > 0) {
//                                    drive.setMotorPowers(detX/10,-detX/10,detX/10,-detX/10);
                                    strafe = "right";
                                } else {
//                                    drive.setMotorPowers(-detX/10,detX/10,-detX/10,detX/10);
                                    strafe = "left";
                                }

//                                this.sleep(20);
                            }


                            if(detY!=targetY) {
                                if (detY > targetY) {
                                     direction = "forward";
//                                    drive.setMotorPowers(0.1,0.1,0.1,0.1); // do something about this later
                                } else {
//                                    drive.setMotorPowers(-0.1,-0.1,-0.1,-0.1);
                                     direction = "backward";
                                }
//                                this.sleep(20);

                            }
                        }






                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop
            telemetry.addData("X direction: ", strafe);
            telemetry.addData("Y direction: ", direction);
            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}