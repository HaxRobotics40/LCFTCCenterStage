package org.firstinspires.ftc.teamcode.drive.opmode.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(group = "comp")
public class Near2Plus0OpMode extends OpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;
    testEOCVpipeline detector = new testEOCVpipeline();
    //    TODO: Use dead wheels
    SampleMecanumDrive drive;
    //TODO: Update Constants to be 100% accurate (ex. wheel radius)
    IMU imu;
    DistanceSensor sensorDistance;
    InputOutput arm;
    int status= 0;
    int itemSector;
    Pose2d startPose;
    double distForward;
    int isBlue = -1;
    int parkSide = -1;
    double output = 0.1;
    Pose2d distanceSensePose;
    TrajectorySequence afterDistSense;
    TrajectorySequence wholeAutoMode;
    @Override
    public void init() {
        arm = new InputOutput(hardwareMap, true, .5, .5);
        vPortal = initVisionPortal();
        arm.setup();
        setupIMU();
        setupDistanceSensor();
        vPortal.setProcessorEnabled(detector, true);
        status = 0;

        drive = new SampleMecanumDrive(hardwareMap);

    }
    public void init_loop() {
        if (gamepad1.dpad_left) {
            parkSide = 1;
        } else if (gamepad1.dpad_right) {
            parkSide = -1;
        }

        if (detector.getColor() == "RED") {
            isBlue = -1;
            startPose = new Pose2d(12,-66, Math.toRadians(270));
            drive.setPoseEstimate(startPose);
        } else if (detector.getColor() == "BLUE") {
            isBlue = 1;
            startPose = new Pose2d(12,66, Math.toRadians(90));
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

    public void start() {
        wholeAutoMode = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, isBlue*43, Math.toRadians((-isBlue*90)+((itemSector-1)*-39.4))))
                .addDisplacementMarker(() -> {
                    arm.ground();
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    arm.releaseLeft();
                    arm.setAngle(1);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(12, 36*isBlue, Math.toRadians(180)))
                .lineTo(new Vector2d(48, 36*isBlue))
                .strafeLeft(((itemSector-1)*5.25)-2)
                .build();
        drive.followTrajectorySequence(wholeAutoMode);

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

        afterDistSense = drive.trajectorySequenceBuilder(distanceSensePose)
                .forward(-output) // this is going to cause an error because when inited it will be an emptypathsegment
                .addDisplacementMarker(() -> {
                    arm.board();
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    arm.release();
                })
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    arm.rest();
                    arm.grab();
                })
                .turn(Math.toRadians(isBlue*90))
                .lineToLinearHeading(new Pose2d(64, isBlue*(36+(parkSide*20)), Math.toRadians(isBlue*90)))
                .build();
        drive.followTrajectorySequence(afterDistSense);
    }
    public void loop() {
        opModeLoop();
    }
    private VisionPortal initVisionPortal() {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.addProcessor(detector);

        return vPortalBuilder.build();
    }
    private void opModeLoop() {
        arm.update();
        drive.update();
        telemetry.addData("power", arm.getPivotPower());
        telemetry.addData("target", arm.getTargetPivotPos());
        telemetry.addData("pose", arm.getPivotPos());
        telemetry.update();
    }

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT; // TODO: Update once it's done being built
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    private void setupDistanceSensor() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }
//    private void driveToLine() {
//        TrajectorySequence traj1;
//        traj1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(12, -43, Math.toRadians(90+((itemSector-1)*-39.4))))
//                .build();
//        drive.followTrajectorySequence(traj1);
//        status++;
//    }
//    //    }
//    private void colorProcess() { // get pose estimate, add second one
////        double redValue =  colorSensor.getNormalizedColors().red;
////        double blueValue = colorSensor.getNormalizedColors().blue;
////
////        teleData("Red Value (0 to 1)", "%4.2f", redValue);
////        teleData("Blue Value (0 to 1)", "%4.2f", blueValue);
////        telemetry.update();
//
////        if (redValue > 0.4 || blueValue > 0.5) {
//            // We found a line (either red or blue)
////            drive.setMotorPowers(0, 0, 0, 0); // Stop the robot
//            status++;
//        if (detector.getColor() == "RED") {
//            isBlue = 0;
//            pose3 = drive.getPoseEstimate();
//        } else if (detector.getColor() == "BLUE") {
//            isBlue = 1;
//            Pose2d thing = drive.getPoseEstimate();
//            pose3 = new Pose2d(thing.getX(), -1*thing.getY(), thing.getHeading()+Math.toRadians(180));
//        }
////        } else {
//            // Continue moving forward if no line is detected
////            Trajectory myTrajectory = drive.trajectoryBuilder(pose2)
////                    .forward(1)
////                    .build();
////            drive.followTrajectory(myTrajectory);
////            pose2 = drive.getPoseEstimate();
////        }
//    }
//    private void crossField() {
//        if (itemSector !=2) {
//            trajCross = drive.trajectorySequenceBuilder(pose3)
//                    .lineToLinearHeading(new Pose2d(12, Math.signum(pose3.getY())*36, 0))
//                    .forward(36)
//                    .strafeRight((itemSector - 2) * 5.25)
//                    .build();
//        } else {
//            trajCross = drive.trajectorySequenceBuilder(pose3)
//                    .lineToLinearHeading(new Pose2d(12, Math.signum(pose3.getY())*36, 0))
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
//
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
//
//    private void outputTelemetry() {
//        // TODO: Also output to .log file.
////        teleLogging("---------April Tag Data----------");
////        aprilTagTelemetry();
//        teleLogging(String.valueOf(itemSector));
//        teleData("status: ", status);
//        teleLogging("---------IMU Data----------");
//        IMUTelemetry();
//        teleLogging("---------Pose Data----------");
////        TODO: Add beysian estimate. Kalman filter.
//        poseTelemetry();
//        teleLogging("---------Color Data----------");
//        colorSensorTelemetry();
//        teleLogging("---------Distance Sensor----------");
//        distanceSensorTelemetry();
//        teleData("parkSide", parkSide);
//    }
//    @SuppressLint("DefaultLocale")
//    private void distanceSensorTelemetry() {
//        teleData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
//    }
//
//    private void colorSensorTelemetry() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        teleData("Red", "%.3f", colors.red);
//        teleData("Green", "%.3f", colors.green);
//        teleData("Blue", "%.3f", colors.blue);
//
//    }
//
//    private void poseTelemetry() {
//        teleLogging(String.format("Estimated Pose: %s", drive.getPoseEstimate()));
//    }
//
//    @SuppressLint("DefaultLocale")
//    private void aprilTagTelemetry() {
//        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//        teleData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                teleLogging(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                teleLogging(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                teleLogging(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                teleLogging(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                teleLogging(String.format("\n==== (ID %d) Unknown", detection.id));
//                teleLogging(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//    }
//    private void IMUTelemetry() {
////        TODO: create IMU Class.
//        // Retrieve Rotational Angles and Velocities
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//
//        teleData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//        teleData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//        teleData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//        teleData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//        teleData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//        teleData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
//    }
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