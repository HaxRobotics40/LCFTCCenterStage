package org.firstinspires.ftc.teamcode.drive.opmode.autoModes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(group = "comp")
public class knoscorenear extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    testEOCVpipeline detector = new testEOCVpipeline();
    //    TODO: Use dead wheels
    SampleMecanumDrive drive;
    //TODO: Update Constants to be 100% accurate (ex. wheel radius)
    IMU imu;

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
//    double detBearing;
//    private final double kP = 0;
//    private final double kI = 0;
//    private final double kD = 0 ;
//    PIDController pid = new PIDController(kP, kI, kD);
    Pose2d boardPose;
    Pose2d scorePoseYellow;
    int parkSide = 0;
    @Override
    public void runOpMode() throws InterruptedException {
//        aprilTagProcessor = initAprilTag();
//        vPortal = initVisionPortal();


        setupIMU();


        drive = new SampleMecanumDrive(hardwareMap);

//        if (gamepad1.dpad_left && gamepad1.left_bumper) {
//            parkSide = 1;
//        } else if (gamepad1.dpad_right && gamepad1.right_bumper) {
//            parkSide = -1;
//        }
//        telemetry.addData("Parking side", parkSide);
//        telemetry.update();
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
//    private VisionPortal initVisionPortal() {
//        vPortalBuilder = new VisionPortal.Builder();
//        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
//        vPortalBuilder.addProcessors(detector, aprilTagProcessor);
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
        }
        telemetry.update();
    }
    private void driveToLine() {
        Trajectory traj1;
            traj1 = drive.trajectoryBuilder(startPose)

                    .strafeLeft(48)
                    .build();
        drive.followTrajectory(traj1);
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
                if (detector.getColor() == "BLUE") {
                    isBlue =1;
                } else {
                    isBlue = 0;
                }
                vPortal.stopStreaming();
            }
        }
    }

    @SuppressLint("DefaultLocale")

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