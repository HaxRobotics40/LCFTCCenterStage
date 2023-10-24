package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.controller.PIDController;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.*;
import android.util.Size;

import java.util.*;

//rr and strucutre layouts for post tuning stuff
// start the op mode in the red location closer to the backboard

@Autonomous
public class testEC extends LinearOpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static int target = 0;
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;

    SampleMecanumDrive drive;
    double detX;
    double detBearing;
    double threshold = 0.2;
    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private double previousError = 0;
    private double integral = 0;
    double power;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagProcessor = initAprilTag();
        vPortal = initVisionPortal(aprilTagProcessor);

        drive = new SampleMecanumDrive(hardwareMap); //try followTrajectoryAsync later?
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        if (opModeIsActive()) {
            vPortal.setProcessorEnabled(aprilTagProcessor, true);
            while (opModeIsActive()) {
//                controller.setPID(p,i,d);
                getVals();
//                double error = controller.getPositionError();
                double error = 0-detX;
                power = pidCorrection(error);
                // power = kP*error;
                telemetry.addData("Error:", error);
                telemetry.addData("detX: ", detX);
                telemetry.addData("Output: ", power);
                telemetry.update();
                if (shouldStopStrafing()) {
                    drive.setMotorPowers(0,0,0,0);
                } else {
                    drive.setMotorPowers(power, -power, power, -power);
                }
            }
        }

    }

    private AprilTagProcessor initAprilTag() {
        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setLensIntrinsics(162, 162, 360, 360);
        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());

        return aprilTagProcessorBuilder.build();
    }

    private VisionPortal initVisionPortal(AprilTagProcessor atp) {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.setCameraResolution(new Size(1920,1080));
        vPortalBuilder.addProcessor(atp);

        return vPortalBuilder.build();
    }
    private void getVals() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 6) {
                detX = detection.ftcPose.x;
                detBearing = detection.ftcPose.bearing;
            }
        }
    }
    private double pidCorrection(double error) {
        double derivative = (error - previousError)/timer.seconds();
        integral += (error*timer.seconds());

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        previousError = error;
        return output;
    }
    public boolean shouldStopStrafing() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 6 && Math.abs(detection.ftcPose.x) < 1) {
                return true;
            }
        }
        return false;
    }


}