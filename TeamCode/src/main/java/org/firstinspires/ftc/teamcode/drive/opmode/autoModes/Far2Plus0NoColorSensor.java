package org.firstinspires.ftc.teamcode.drive.opmode.autoModes;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Autonomous(group = "comp")
public class Far2Plus0NoColorSensor extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;
    AprilTagProcessor aprilTagProcessor;
    testEOCVpipeline detector = new testEOCVpipeline();
    SampleMecanumDrive drive;
    IMU imu;
    DistanceSensor sensorDistance;
    InputOutput arm;
    double output;
    TrajectorySequence wholeAutoMode;
    int lastTime;
    int thisTime;
    int status;
    int itemSector;
    Pose2d startPose;
    double distForward;
    int isBlue = -1;
    TrajectorySequence afterDistSense;
    Pose2d distanceSensePose;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    int parkSide = -1;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = new InputOutput(hardwareMap, true, .5, .5);
        vPortal = initVisionPortal();
        arm.rest();

        setupIMU();


        setupDistanceSensor();


        drive = new SampleMecanumDrive(hardwareMap);

//        drive.setPoseEstimate(new Pose2d(-36,-60, Math.toRadians(270)));
        if (opModeInInit()) {
            while (opModeInInit()) {
                if (gamepad1.dpad_left) {
                    parkSide = 1;
                } else if (gamepad1.dpad_right) {
                    parkSide = -1;
                }

                if (detector.getColor() == "RED") {
                    isBlue = -1;
                    startPose = new Pose2d(-36,-66, Math.toRadians(270));
                    drive.setPoseEstimate(startPose);
                } else if (detector.getColor() == "BLUE") {
                    isBlue = 1;
                    startPose = new Pose2d(-36,66, Math.toRadians(90));
                    drive.setPoseEstimate(startPose);
                }

                if (detector.locationInt() != -1) {
                    itemSector = detector.locationInt();
                }
                telemetry.addData("Parking side", parkSide);
                telemetry.addData("Location", detector.locationInt());
                telemetry.addData("Color", detector.getColor());
                telemetry.addData("Pose estimate", drive.getPoseEstimate().toString());
                telemetry.update();
            }
        }
        waitForStart();
        if (opModeIsActive()) {
            wholeAutoMode = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-36, isBlue*43, Math.toRadians((-isBlue*90)+((itemSector-1)*-39.4))))
                    .addDisplacementMarker(() -> {
                        arm.ground();
                    })
                    .waitSeconds(0.5)
                    .addDisplacementMarker(() -> {
                        arm.releaseLeft();
                        arm.setAngle(1);
//                        arm.grab();
                    })
                    .lineToLinearHeading(new Pose2d(-36, 36*isBlue, Math.toRadians(180)))
                    .lineTo(new Vector2d(48, 36*isBlue))
                    .strafeLeft(((itemSector-1)*5.25)-2)
                    .build();


            while (opModeIsActive()) {
                opModeLoop();
            }
        }

    }
    //
    private VisionPortal initVisionPortal() {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.addProcessor(detector);
        vPortal.setProcessorEnabled(detector, true);

//        vPortalBuilder.setCameraResolution(new Size (1920,1080));

        return vPortalBuilder.build();
    }

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void setupDistanceSensor() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }
    private void opModeLoop() {
        switch (status) {
            case 0: drive.followTrajectorySequence(wholeAutoMode);
                status++;
                break;
            case 1:
                double wantedDistance = 3; // how far away you want the robot to go
                double thresholdDistanceInches = 0.1;

                distForward = sensorDistance.getDistance(DistanceUnit.INCH);

                if (sensorDistance.getDistance(DistanceUnit.INCH) != DistanceUnit.infinity) {
                    distForward = sensorDistance.getDistance(DistanceUnit.INCH);
                    if (Math.abs(distForward-wantedDistance) > thresholdDistanceInches) {
                        output = wantedDistance - distForward;
                    }
                } else {
                    output = 0.1;
                }
                Pose2d lastPos = drive.getPoseEstimate();
                distanceSensePose = (new Pose2d(72-(distForward+16.52), lastPos.getY(), lastPos.getHeading()));
                status++;
                break;
            case 2:
                afterDistSense = drive.trajectorySequenceBuilder(distanceSensePose)
                        .forward(-output) // this is going to cause an error because when inited it will be an emptypathsegment
                        .addDisplacementMarker(() -> {
                            arm.board();
                        })
                        .waitSeconds(0.5)
                        .addDisplacementMarker(() -> {
                            arm.release();
                            arm.rest();
                            arm.grab();
                        })
                        .turn(isBlue*90)
                        .lineToLinearHeading(new Pose2d(64, isBlue*(36+(parkSide*20)), Math.toRadians(isBlue*90)))
                        .build();
                status++;
                break;
            case 3:
                drive.followTrajectorySequence(afterDistSense);
                status++;
                break;
        }
    }
//    private void driveToLine() {
//        TrajectorySequence traj1;
//        traj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-36, -43, Math.toRadians(90+((itemSector-1)*-39.4))))
//                .build();
//        drive.followTrajectorySequence(traj1);
//        pose2 = drive.getPoseEstimate();
//        status++;
//    }
//    //    }
//
//    private void crossField() {
//        if (itemSector !=2) {
//            trajCross = drive.trajectorySequenceBuilder(pose3)
//                    .lineToLinearHeading(new Pose2d(-36, Math.signum(pose3.getY())*36, 0))
//                    .forward(36)
//                    .strafeRight((itemSector - 2) * 5.25)
//                    .build();
//        } else {
//            trajCross = drive.trajectorySequenceBuilder(pose3)
//                    .lineToLinearHeading(new Pose2d(-36, Math.signum(pose3.getY())*36, 0))
//                    .forward(36)
//                    .build();
//        }
//        drive.followTrajectorySequence(trajCross);
//        boardPose = drive.getPoseEstimate();
//        status++;
//    }
//    private void park() {
//        if (isBlue == 1) {
//            TrajectorySequence park = drive.trajectorySequenceBuilder(scorePoseYellow)
//                    .strafeLeft(24)
//                    .build();
//            drive.followTrajectorySequence(park);
//        } else {
//            TrajectorySequence park = drive.trajectorySequenceBuilder(scorePoseYellow)
//                    .strafeRight(24)
//                    .build();
//            drive.followTrajectorySequence(park);
//        }
//    }
//    private void dropPixel() {
//        arm.ground();
//        arm.releaseLeft();
//        arm.rest();
//        status++;
//    }
//    private void scorePixel() {
//        arm.board();
//        arm.releaseRight();
//        arm.rest();
//    }
//
//    private void runPieceDetector() {
//        // R is 0, M is 1, L is 2
//        vPortal.setProcessorEnabled(detector, true);
//        boolean stop = false;
//        while (!stop) {
//            if (detector.locationInt() != -1) {
//                itemSector = detector.locationInt();
//                //TODO: run a couple times, area of mask is sufficient, find most common of 20 or so frames
//                stop = true;
//                status++;
//                vPortal.stopStreaming();
//            }
//        }
//    }
//    private void fixDistance() {
//        double wantedDistance = 12.75; // how far away you want the robot to go
//
//        double thresholdDistanceInches = 0.1;
//
//        distForward = sensorDistance.getDistance(DistanceUnit.INCH);
//
//        if (sensorDistance.getDistance(DistanceUnit.INCH) != DistanceUnit.infinity) {
//            distForward = sensorDistance.getDistance(DistanceUnit.INCH);
//        } else {
//            distForward = wantedDistance;
//            teleLogging("Infinity Distance detected");
//            scorePoseYellow = drive.getPoseEstimate();
//        }
//        if (distForward != wantedDistance) {
//            double output = wantedDistance - distForward;
//            Trajectory Disttraj = drive.trajectoryBuilder(boardPose)
//                    .forward(output)
//                    .build();
//
//
//            String StrMotorPower = String.valueOf(output);
//            teleLogging("wanted Motor Power:" + StrMotorPower);
//            drive.followTrajectory(Disttraj);
//        }
//        if ((distForward >= wantedDistance - thresholdDistanceInches) &&
//                (distForward <= wantedDistance + thresholdDistanceInches)) {
//            teleLogging("Achieved location");
//            drive.setMotorPowers(0,0,0,0);
//            status++;
////            Pose2d scorePoseYellow = drive.getPoseEstimate();
//        }
//    }
//    private void colorProcess() { // get pose estimate, add second one
//
//        status++;
//        if (Objects.equals(detector.getColor(), "RED")) {
//            isBlue = 0;
//            pose3 = drive.getPoseEstimate();
//        } else if (Objects.equals(detector.getColor(), "BLUE")) {
//            isBlue = 1;
//            Pose2d thing = drive.getPoseEstimate();
//            pose3 = new Pose2d(thing.getX(), -1*thing.getY(), thing.getHeading()+Math.toRadians(180));
//        }
////
//    }

    private void outputTelemetry() {
        // TODO: Also output to .log file.
        teleLogging(String.valueOf(itemSector));
        teleData("status: ", status);
        teleLogging("---------IMU Data----------");
        IMUTelemetry();
        teleLogging("---------Pose Data----------");
//        TODO: Add beysian estimate. Kalman filter.
        poseTelemetry();
        teleLogging("---------Color Data----------");

        teleLogging("---------Distance Sensor----------");
        distanceSensorTelemetry();
//        teleData("parkSide", parkSide);
    }

    @SuppressLint("DefaultLocale")
    private void distanceSensorTelemetry() {
        teleData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
    }

    private void poseTelemetry() {
        teleLogging(String.format("Estimated Pose: %s", drive.getPoseEstimate()));
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