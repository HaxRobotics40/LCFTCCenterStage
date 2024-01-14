package org.firstinspires.ftc.teamcode.drive.opmode.autoModes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(group = "comp")
public class redfarstraight2p extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;
    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    testEOCVpipeline detector = new testEOCVpipeline();
    //    TODO: Use dead wheels
    SampleMecanumDrive drive;
    //TODO: Update Constants to be 100% accurate (ex. wheel radius)
    IMU imu;
    DistanceSensor sensorDistance;
//    InputOutput arm;
    int status;
    int itemSector;
    Pose2d startPose = new Pose2d(-36,-60, Math.toRadians(90));
    double detX;
    double distForward;
    int isBlue = -1;
    Pose2d pose2;
    Pose2d pose3;
    TrajectorySequence trajCross;
    double detBearing;
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0 ;
    PIDController pid = new PIDController(kP, kI, kD);
    Servo pixel;
    Pose2d parkPose;
    Pose2d scorePoseYellow;
    int parkSide = 0;
    @Override
    public void runOpMode() throws InterruptedException {
//        aprilTagProcessor = initAprilTag();
//        vPortal = initVisionPortal();
//        pixel = hardwareMap.get(Servo.class, "pixel");
//        pixel.setPosition(0.9);

        setupIMU();


        drive = new SampleMecanumDrive(hardwareMap);
        // TODO: Fix Drive Constants physical measurements!!!
//        TODO: Move Reverse to here.

//        Thread telemetryThread = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
//                    outputTelemetry();
//
//                    try {
//                        Thread.sleep(10); // Introducing a small delay to prevent excessive updates
//                    } catch (InterruptedException e) {
//                        Thread.currentThread().interrupt();
//                    }
//                }
//            }
//        });
//        if (gamepad1.dpad_left && gamepad1.left_bumper) {
//            parkSide = 1;
//        } else if (gamepad1.dpad_right && gamepad1.right_bumper) {
//            parkSide = -1;
//        }
        telemetry.update();
        drive.setPoseEstimate(new Pose2d(-36,-60, Math.toRadians(90)));
        waitForStart();

//        telemetryThread.start(); // Starting telemetry thread

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                opModeLoop();
            }
        }



//        telemetryThread.interrupt(); // Make sure to interrupt the telemetry thread when opMode is no longer active
    }

//    private AprilTagProcessor initAprilTag() {
//        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
//        aprilTagProcessorBuilder.setLensIntrinsics(957.381,957.381,924.159,537.109);
//
//        return aprilTagProcessorBuilder.build();
//    }
//    //
//    private VisionPortal initVisionPortal() {
//        vPortalBuilder = new VisionPortal.Builder();
//        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
//        vPortalBuilder.addProcessors(detector, aprilTagProcessor);
//        vPortalBuilder.setCameraResolution(new Size (1920,1080));
//
//        return vPortalBuilder.build();
//    }

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    private void opModeLoop() {
        switch(status) {
            case 0: driveToLine();
                break;
//            case 1: driveToLine();
//                break;
        }
    }
    private void driveToLine() {
        TrajectorySequence traj1;
            traj1 = drive.trajectorySequenceBuilder(new Pose2d(-36, -66, Math.toRadians(0)))
                    .strafeTo(new Vector2d(-36, -62))
                    .lineToConstantHeading(new Vector2d(66, -62))
                    .build();
        drive.followTrajectorySequence(traj1);
//        pose2 = drive.getPoseEstimate();
        status++;
    }
    //    }



    private void runPieceDetector() {
        // R is 0, M is 1, L is 2
        vPortal.setProcessorEnabled(detector, true);
        boolean stop = false;
        while (!stop) {
            if (detector.locationInt() != -1) {
                itemSector = detector.locationInt();
                //TODO: run a couple times, area of mask is sufficient, find most common of 20 or so frames
                stop = true;
                status++;
                vPortal.stopStreaming();
            }
        }
    }
    @SuppressLint("DefaultLocale")
    private void distanceSensorTelemetry() {
        teleData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
    }

    @SuppressLint("DefaultLocale")
    private void aprilTagTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        teleData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                teleLogging(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                teleLogging(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                teleLogging(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                teleLogging(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                teleLogging(String.format("\n==== (ID %d) Unknown", detection.id));
                teleLogging(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }
    private void IMUTelemetry() {
//        TODO: create IMU Class.
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        teleData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        teleData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        teleData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        teleData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        teleData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        teleData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
    }
    private void teleLogging(String s) {
        telemetry.addLine(s);
        RobotLog.d(s);
    }
    private void teleData(String s, String format, Object... args) {
        telemetry.addData(s, format, args);
        String stringArguments = String.format(format, args);
        RobotLog.d(s + ": " + stringArguments);
    }
    private void teleData(String s, Object... args) {
        telemetry.addData(s, args);
        String stringArguments = String.valueOf(args);
        RobotLog.d(s + ": " + stringArguments);
    }

}