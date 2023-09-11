package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class AutonomousMode extends LinearOpMode {
    WebcamName webcamName;
    OpenCvCamera webcam;
    double TheLocation;
    SampleMecanumDrive drive;
    // TODO: Update this.
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    DcMotorEx FE;
    DcMotorEx LE;
    DcMotorEx RE;

    @Override
    public void runOpMode() throws InterruptedException {

    }


    public void initRobot() {
        
    }
}