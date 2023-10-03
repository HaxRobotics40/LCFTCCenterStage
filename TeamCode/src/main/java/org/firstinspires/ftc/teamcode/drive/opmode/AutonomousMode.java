package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.*;
import com.qualcomm.robotcore.util.RobotLog;


@Autonomous
public class AutonomousMode extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;

//    SampleMecanumDrive drive;
//    TODO: Use dead wheels
//    TODO: Update Constants to be 100% accurate (ex. wheel radius)
    IMU imu;

//    NormalizedColorSensor colorSensor;

    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() throws InterruptedException {
//        aprilTagProcessor = initAprilTag();
//        vPortal = initVisionPortal(aprilTagProcessor);
//        telemetry.setAutoClear(false);

        setupIMU();

//        setupColorSensor();
        setupDistanceSensor();

//        drive = new SampleMecanumDrive(hardwareMap);
        // TODO: Fix Drive Constants physical measurements!!!
//        TODO: Move Reverse to here.

        Thread telemetryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                    outputTelemetry();
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

        if (opModeIsActive()) {
            while (opModeIsActive()) {
//               opModeLoop();
            }
        }

        telemetryThread.interrupt(); // Make sure to interrupt the telemetry thread when opMode is no longer active
    }

    private void opModeLoop() {

    }

//    private AprilTagProcessor initAprilTag() {
//        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
//
//        return aprilTagProcessorBuilder.build();
//    }

//    private VisionPortal initVisionPortal(AprilTagProcessor atp) {
//        vPortalBuilder = new VisionPortal.Builder();
//        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
//        vPortalBuilder.addProcessor(atp);
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

//    private void setupColorSensor() {
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }
//
//        colorSensor.setGain(15);
//        // TODO: FInd Color Sensor docs on best gain
//    }

    private void setupDistanceSensor() {
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }


    private void outputTelemetry() {
        // TODO: Also output to .log file.
        addLine("---------April Tag Data----------");
//        aprilTagTelemetry();
        addLine("---------IMU Data----------");
        IMUTelemetry();
        addLine("---------Pose Data----------");
//        TODO: Add beysian estimate. Kalman filter.
        poseTelemetry();
//        addLine("---------Color Data----------");
//        colorSensorTelemetry();
        addLine("---------Distance Sensor----------");
//        distanceSensorTelemetry();
    }

    private void distanceSensorTelemetry() {
//        addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
    }

//    private void colorSensorTelemetry() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
//    }

    private void poseTelemetry() {
//        addLine(String.format("Estimated Pose: %s", drive.getPoseEstimate().toString()));
    }

    @SuppressLint("DefaultLocale")
//    private void aprilTagTelemetry() {
//            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//            addData("# AprilTags Detected", currentDetections.size());
//
//            // Step through the list of detections and display info for each one.
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                    addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                } else {
//                    addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                    addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                }
//            }   // end for() loop
//    }

    private void IMUTelemetry() {
//        TODO: create IMU Class.
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.update();
    }

    private void addLine(String log) {
        telemetry.addLine(log);
        RobotLog.d("HAX40:" + log);
    }
    private void addData(String log, Object... args) {
        telemetry.addData(log, args);
        RobotLog.d("HAX40:" + log, args);
    }
}
