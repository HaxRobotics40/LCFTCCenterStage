package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.*;

import java.util.*;

//rr and strucutre layouts for post tuning stuff
// start the op mode in the red location closer to the backboard

@Autonomous
public class testEC extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;
    boolean proceed = false;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    TrajectorySequence trajSeq;

    SampleMecanumDrive drive;
    int targetTag;
    double detX;
    double threshold = 0.4;
    Pose2d startPose = new Pose2d(36,-60,Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagProcessor = initAprilTag();
        vPortal = initVisionPortal(aprilTagProcessor);

        drive = new SampleMecanumDrive(hardwareMap); //try followTrajectoryAsync later?
        targetTag = 3;

        waitForStart();


        if (opModeIsActive()) {
            vPortal.setProcessorEnabled(aprilTagProcessor, true);
            while (opModeIsActive()) {
                if(!proceed) {
                    aprilTagTelemetry();
                    telemetry.update();
                } else {
                    drive.setMotorPowers(0, 0, 0, 0);
                    vPortal.close();
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
        vPortalBuilder.addProcessor(atp);

        return vPortalBuilder.build();
    }
    @SuppressLint("DefaultLocale")
    private void aprilTagTelemetry() {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            for (AprilTagDetection det : currentDetections) {
                if (det.id % 3 == targetTag || det.id % 3 == targetTag-3) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", det.ftcPose.x, det.ftcPose.y, det.ftcPose.z));
                    detX = det.ftcPose.x;
                    if(currentDetections.size() != 0) {
                        if ((detX > 0 + threshold || detX < 0 - threshold) && !proceed) {
                            if (detX > 0 + threshold) {
                                drive.setMotorPowers(0.2, -0.2, 0.2, -0.2);
                            } else {
                                drive.setMotorPowers(-0.2, 0.2, -0.2, 0.2);
                            }
                        } else {
                            proceed = true;
                        }
                    } else {
                        drive.setMotorPowers(0,0,0,0);
                    }
                }
            }

    }
}