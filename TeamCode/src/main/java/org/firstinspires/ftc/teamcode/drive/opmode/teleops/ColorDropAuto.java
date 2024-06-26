package org.firstinspires.ftc.teamcode.drive.opmode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//@Disabled
@TeleOp(group = "drive")
public class ColorDropAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Servo pixel = hardwareMap.get(Servo.class, "pixel");
//        pixel.setPosition(1);
        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            if (gamepad1.x) {
                pivot.setPower(1); }else if (gamepad1.y) {
                pivot.setPower(-1); }





//                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
//                telemetry.addData("Servo Position: ", pixel.getPosition());
                telemetry.addData("pos", drive.getWheelPositions());
                telemetry.addData("Velocities", drive.getWheelVelocities());
                telemetry.update();

        }
    }
}
