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
@Autonomous (group= "comp")
public class Far2Plus0OpMode extends OpMode {
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
    public void start() {
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
        drive.followTrajectorySequenceAsync(wholeAutoMode);
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
        drive.followTrajectorySequenceAsync(afterDistSense);
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

