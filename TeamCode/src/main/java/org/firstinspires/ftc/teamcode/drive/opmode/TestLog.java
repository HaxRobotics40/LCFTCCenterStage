package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;

import com.qualcomm.robotcore.util.RobotLog;
import com.arcrobotics.ftclib.controller.*;


@Autonomous
public class TestLog extends LinearOpMode {
//    VisionPortal.Builder vPortalBuilder;
//    VisionPortal vPortal;
//
//    AprilTagProcessor aprilTagProcessor;
//    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    testEOCVpipeline detector = new testEOCVpipeline();
    //    TODO: Use dead wheels
    SampleMecanumDrive drive;
    //TODO: Update Constants to be 100% accurate (ex. wheel radius)
    IMU imu;
    double start = System.currentTimeMillis();

    pidf.setP(0.37);
    pidf.setI(0.05);
    pidf.setD(1.02);

    // get our gain constants
    float kP = pidf.getP();
    float kI = pidf.getI();
    float kD = pidf.getD();
    float kD = pidf.getD();
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    PIDController pid = new PIDController(kP, kI, kD);
    PDController pd = new PDController(kP, kD);
    PController p = new PController(kP);



    NormalizedColorSensor colorSensor;


    DistanceSensor sensorDistance;
    int status;
    int itemSector;
    Pose2d startPose = new Pose2d(-36,-60, Math.toRadians(90));
    double detX;
    double distForward;

    @Override
    public void runOpMode() throws InterruptedException {
//        aprilTagProcessor = initAprilTag();
//        vPortal = initVisionPortal();

        setupIMU();

//        setupColorSensor();
        setupDistanceSensor();

        drive = new SampleMecanumDrive(hardwareMap);
        // TODO: Fix Drive Constants physical measurements!!!
//        TODO: Move Reverse to here.

        Thread telemetryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                    distanceSensorTelemetry();
                    telemetry.update();
                    try {
                        Thread.sleep(10); // Introducing a small delay to prevent excessive updates
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }

            }
        });

        waitForStart();

        telemetryThread.start(); // Starting telemetry thread

        teleLogging("going to Barrier of distance, attem:15");

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                goToBarrierOfDistance(12);
                distanceSensorTelemetry();
                telemetry.update();
            }
        }



        telemetryThread.interrupt(); // Make sure to interrupt the telemetry thread when opMode is no longer active
    }

    private void setupIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    private void setupDistanceSensor() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    private void goToBarrierOfDistance(double distance) {


        float wantedDistance = 12.75F; // how far away you want the robot to go

        float thresholdDistanceinches = 0.1F;

        distForward = sensorDistance.getDistance(DistanceUnit.INCH);

        if (sensorDistance.getDistance(DistanceUnit.INCH) != DistanceUnit.infinity) {
            distForward = sensorDistance.getDistance(DistanceUnit.INCH);
        } else {
            distForward = wantedDistance;
            teleLogging("Infinity Distance detected");
        }
        if (distForward != wantedDistance) {
            double motorPower = 0.001 * Math.pow((distForward - wantedDistance),3F);
            String StrMotorPower = String.valueOf(motorPower);
            teleLogging("wanted Motor Power:" + StrMotorPower);
            drive.setMotorPowers(motorPower, motorPower, motorPower, motorPower);
        }
        if ((distForward >= wantedDistance - thresholdDistanceinches) &&
                (distForward <= wantedDistance + thresholdDistanceinches))
        {
            teleLogging("Achieved location");
            drive.setMotorPowers(0,0,0,0);
            return;
        }
    }


    @SuppressLint("DefaultLocale")
    private void distanceSensorTelemetry() {
        teleLogging(String.format("%.01f in.", sensorDistance.getDistance(DistanceUnit.INCH)));
    }
    private void teleLogging(String s) {
        telemetry.addLine(s);
        RobotLog.d(s);
    }
    private void teleData(String s, String format, Object... args) {
        telemetry.addData(s, format, args);
        String stringArguments = String.format(format, args);
        RobotLog.d(s + ": ", args);
    }
    private void teleData(String s, Object... args) {
        telemetry.addData(s, args);
        String stringArguments = String.valueOf(args);
        RobotLog.d(s + ": " + stringArguments);
    }




}