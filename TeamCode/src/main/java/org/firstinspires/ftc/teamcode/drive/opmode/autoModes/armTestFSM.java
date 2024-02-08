package org.firstinspires.ftc.teamcode.drive.opmode.autoModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(group = "comp")
//@Disabled
public class armTestFSM extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;
    testEOCVpipeline detector = new testEOCVpipeline();
    //    TODO: Use dead wheels
    SampleMecanumDrive drive;
    //TODO: Update Constants to be 100% accurate (ex. wheel radius)
    IMU imu;
    public enum STATES{
        INIT,
        DRIVE,
        DROP,
        STOP;
    }
    DistanceSensor sensorDistance;
    InputOutput arm;
    int status= 0;
    int itemSector;
    Pose2d startPose= new Pose2d(-36, -60, Math.toRadians(90));
    int isBlue = -1;
    int parkSide = -1;
    double output = 0.1;
    Servo clawL;
    Servo clawR;
    Servo wrist;

    private STATES previousState = STATES.INIT;
    private STATES currentState = STATES.INIT;
    TrajectorySequence wholeAutoMode;
    @Override
    public void runOpMode() {
        arm = new InputOutput(hardwareMap, true, .5, .5);
//        vPortal = initVisionPortal();
        arm.setup();
        setupIMU();
//        setupDistanceSensor();
//        vPortal.setProcessorEnabled(detector, true);

        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        wrist = hardwareMap.get(Servo.class, "wrist");

        status = 0;

        drive = new SampleMecanumDrive(hardwareMap);

        if (opModeInInit()) {
            while (opModeInInit()) {
//                if (detector.getColor() == "RED") {
//                    isBlue = -1;
//                    startPose = new Pose2d(12,-66, Math.toRadians(270));
//                    drive.setPoseEstimate(startPose);
//                } else if (detector.getColor() == "BLUE") {
//                    isBlue = 1;
//                    startPose = new Pose2d(12,66, Math.toRadians(90));
//                    drive.setPoseEstimate(startPose);
//                }
//
//                if (detector.locationInt() != -1) {
//                    itemSector = detector.locationInt();
//                }
//
//                if (detector.getColor() == "BLUE") {
//                    isBlue = 1;
//                } else {
//                    isBlue = -1;
//                }
                wholeAutoMode = drive.trajectorySequenceBuilder(startPose)
                        .forward(-4)
                        .strafeLeft(4)
                        .build();
//                telemetry.addData("Parking side", parkSide);
//                telemetry.addData("Location", detector.locationInt());
//                telemetry.addData("is it blue?", isBlue);
                telemetry.addData("Pose estimate", drive.getPoseEstimate().toString());
                telemetry.update();
            }
        }
        waitForStart();
        currentState = STATES.DRIVE;
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                switch (currentState) {
                    case INIT:
                        break;
                    case DRIVE:
                        if (previousState != currentState) {
                            // everything in here will run once when the state switches

                            drive.followTrajectorySequenceAsync(wholeAutoMode);
                            previousState = STATES.DRIVE;
                        } else if (!drive.isBusy()) {
                            currentState = STATES.DROP;
                        }
                        break;
                    case DROP:
                        if (previousState != currentState) {
                            arm.ground();
                            previousState = STATES.DROP;
                        } else if (arm.getTargetPivotPos() == 500 && arm.atAngle()) {
                            currentState = STATES.STOP;
                        }
                        break;
                    case STOP:
                        break;
                }
                drive.update();
                arm.update();
            }
        }
    }

    private VisionPortal initVisionPortal() {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.addProcessor(detector);

        return vPortalBuilder.build();
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