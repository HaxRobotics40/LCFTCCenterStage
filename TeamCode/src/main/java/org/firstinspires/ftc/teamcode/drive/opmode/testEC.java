package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.roadrunner.geometry.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.*;

import java.util.*;

//rr and strucutre layouts for post tuning stuff

@Autonomous
public class testEC extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    TrajectorySequence trajSeq;

    SampleMecanumDrive drive;
    IMU imu;
    int targetTag;
    double detX;
    boolean positionYes = false;
    double detY;
    Pose2d startPose = new Pose2d(60,45,Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagProcessor = initAprilTag();
        vPortal = initVisionPortal(aprilTagProcessor);

        setupIMU();
        trajSeqSetup();

        drive = new SampleMecanumDrive(hardwareMap); //try followTrajectoryAsync later?
        targetTag = (int) Math.ceil(3*Math.random());
        telemetry.addData("Randomized Tag Choice: ", targetTag);

        waitForStart();
        if (opModeIsActive()) {

            drive.followTrajectorySequence(trajSeq);
            Thread telemetryThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                        endTickTelemetry();
                        try {
                            Thread.sleep(10); // Introducing a small delay to prevent excessive updates
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }
            });

        }

    }

    private void trajSeqSetup() {

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    // outline searches

                })
                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    while(!positionYes) {
                        aprilTagTelemetry();
                        telemetry.update();
                    }
                })
                .build();


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

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void endTickTelemetry() {
        aprilTagTelemetry();
        IMUTelemetry();
    }

    @SuppressLint("DefaultLocale")
    private void aprilTagTelemetry() {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {


                List<AprilTagDetection> allDets = aprilTagProcessor.getDetections();
                for (AprilTagDetection det : allDets) {

                    // targetTag isolated
                    if (det.id % 3 == targetTag || det.id % 3 == targetTag-3) {

                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                        detX = det.ftcPose.x;
                        detY = det.ftcPose.y;

                        if(detX !=0) {
                            if (detX > 0) {
                                drive.setMotorPowers(detX/10,-detX/10,detX/10,-detX/10);
                            } else {
                                drive.setMotorPowers(-detX/10,detX/10,-detX/10,detX/10);
                            }

                        }
                        if(detY != 12) {
                            if (detX > 12) {
                                drive.setMotorPowers(0.1,0.1,0.1,0.1); // do something about this later
                            } else {
                                drive.setMotorPowers(-0.1,-0.1,-0.1,-0.1);
                            }
                        }

                        if(detX ==0 && detY ==12) {
                            positionYes = true;
                        }


                    }
                }

            }


    }




    private void IMUTelemetry() {
//        TODO: create IMU Class.
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.update();
    }
}