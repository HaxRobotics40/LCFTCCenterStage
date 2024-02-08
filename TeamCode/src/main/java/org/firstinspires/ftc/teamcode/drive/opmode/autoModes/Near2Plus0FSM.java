package org.firstinspires.ftc.teamcode.drive.opmode.autoModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(group = "comp")
@Config
public class Near2Plus0FSM extends LinearOpMode {
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
    Pose2d startPose = new Pose2d(0,0,0);
    double distForward = 0;
    int isBlue = -1;
    int parkSide = -1;
    double output = 0.1;
    public static double delayTime;
    Pose2d distanceSensePose;
    Trajectory driveToLine;
    TrajectorySequence driveToBoard;
    Trajectory e;
    TrajectorySequence park;
//    TrajectorySequence afterDistSense;
//    TrajectorySequence wholeAutoMode;
    public enum STATES {
        INIT,
        ALIGN_LINE,
        SCORE_PURPLE,
        DROP,
        REST,
        ALIGN_BOARD,
        DELAY,
        DISTANCE_SENSE,
        SCORE_YELLOW,
        DROP_YELLOW,
        RESET,
        PARK,
        STOP;
    }
    private STATES previousState = STATES.INIT;
    private STATES currentState = STATES.INIT;
    @Override
    public void runOpMode() throws InterruptedException {


        if (opModeInInit()) {
            arm = new InputOutput(hardwareMap, true, .5, .5);
            vPortal = initVisionPortal();
            arm.setup();
            setupIMU();
            setupDistanceSensor();
            vPortal.setProcessorEnabled(detector, true);
            status = 0;

            drive = new SampleMecanumDrive(hardwareMap);
            while (opModeInInit()) {
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
                driveToLine = drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(12, isBlue*43, Math.toRadians((-isBlue*90)+((itemSector-1)*-39.4))))
                        .build();

                driveToBoard = drive.trajectorySequenceBuilder(driveToLine.end())
                        .lineToLinearHeading(new Pose2d(12, 36*isBlue, Math.toRadians(180)))
                        .lineTo(new Vector2d(48, 36*isBlue))
                        .strafeLeft(((itemSector-1)*5.25)-2)
                        .build();




                telemetry.addData("Parking side", parkSide);
                telemetry.addData("Location", detector.locationInt());
                telemetry.addData("Color", detector.getColor());
                telemetry.addData("Pose estimate", drive.getPoseEstimate().toString());
                telemetry.update();
            }
        }

        waitForStart();
        if (opModeIsActive()) {
            currentState = STATES.ALIGN_LINE;
            while (opModeIsActive()) {
                opModeLoop();
            }
        }
    }
    private VisionPortal initVisionPortal() {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.addProcessor(detector);

        return vPortalBuilder.build();
    }
    private void opModeLoop() {
        switch(currentState) {
            case INIT:
                break;
            case ALIGN_LINE:
                if (previousState != currentState) {
                    // everything in here will run once when the state switches

                    drive.followTrajectoryAsync(driveToLine);
                    previousState = STATES.ALIGN_LINE;
                } else if (!drive.isBusy()) {
                    currentState = STATES.SCORE_PURPLE;
                }
                break;
            case SCORE_PURPLE:
                if (previousState != currentState) {
                    double groundTime = 0;
                    // everything in here will run once when the state switches
                    arm.ground();

                    // it never gets here because it only runs once, idk how to fix this. multithreading?

                    previousState = STATES.SCORE_PURPLE;
                } else {
                    currentState = STATES.DROP;
                }
                break;
            case DROP:
                if (previousState!=currentState) {
                    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    while (timer.time() < 0.5) {
                        arm.releaseLeft();
                    }
                        arm.out();
                    previousState = STATES.DROP;
                } else {
                    currentState = STATES.REST;
                }
                break;
            case REST:
                if (previousState!=currentState) {
                    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    while (!arm.atAngle()) {}
                    previousState = STATES.REST;
                } else {
                    currentState = STATES.DELAY;
                }
                break;
            case DELAY:
                if (previousState != currentState) {
                    ElapsedTime delayTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    if (delayTimer.time() > delayTime) {
                        previousState = STATES.DELAY;
                    }
                } else if (!drive.isBusy()) {
                    currentState = STATES.ALIGN_BOARD;
                }
                break;
            case ALIGN_BOARD:
                if (previousState != currentState) {

                    drive.followTrajectorySequenceAsync(driveToBoard);
                    previousState = STATES.ALIGN_BOARD;
                } else if (!drive.isBusy()) {
                    currentState = STATES.SCORE_YELLOW;
                }
                break;
            case DISTANCE_SENSE:
                if (previousState != currentState) {
                    double wantedDistance = 3; // how far away you want the robot to go
                    double thresholdDistanceInches = 0.1;

                    distForward = sensorDistance.getDistance(DistanceUnit.INCH);

                    if (sensorDistance.getDistance(DistanceUnit.INCH) != DistanceUnit.infinity && sensorDistance.getDistance(DistanceUnit.INCH) != 0) {
                        distForward = sensorDistance.getDistance(DistanceUnit.INCH);
                        if (Math.abs(distForward - wantedDistance) > thresholdDistanceInches) {
                            output = wantedDistance - distForward;
                        }
                    } else {
                        output = 0.1;
                    }

                    Pose2d lastPos = drive.getPoseEstimate();
                    distanceSensePose = (new Pose2d(72 - (distForward + 16.52), lastPos.getY(), lastPos.getHeading()));
                    e = drive.trajectoryBuilder(driveToBoard.end()) // TODO change this back
                            .forward(output)
                            .build();
                    drive.followTrajectoryAsync(e);
                    previousState = STATES.DISTANCE_SENSE;
                }
                if (!drive.isBusy()) {

                    currentState = STATES.SCORE_YELLOW;
                }
                break;
            case SCORE_YELLOW:
                if (previousState != currentState) {
                    arm.wristOut();
                    arm.frontBoard();
                } else {
                    currentState = STATES.PARK;
                }
                break;
            case DROP_YELLOW:
                if (previousState != currentState) {
                    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    while (timer.time() < 0.25) {}
                    arm.release();
                    while (timer.time() < 0.75) {}
                    arm.rest();
                } else {
                    currentState = STATES.PARK;
                }
                break;
            case PARK:
                if (previousState != currentState) {
                    park = drive.trajectorySequenceBuilder(e.end())
                            .turn(Math.toRadians(isBlue*90))
                            .lineToLinearHeading(new Pose2d(64, isBlue*(36+(parkSide*20)), Math.toRadians(isBlue*90)))
                            .build();
                    drive.followTrajectorySequenceAsync(park);
                    previousState = STATES.PARK;
                } else  if (!drive.isBusy()) {
                    currentState = STATES.STOP;
                }
                break;
            case STOP:
                arm.wristIn();
                break;
        }

        // see if this works, i kind of doubt some arm functions will work (like the timer states, those seem weird)


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