package org.firstinspires.ftc.teamcode.drive.opmode;

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
@TeleOp(group = "drive")
public class testRobotCentricWithLinSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo clawL = hardwareMap.get(Servo.class, "clawL");
        Servo clawR = hardwareMap.get(Servo.class, "clawR");
        Servo hook = hardwareMap.get(Servo.class, "hook");
        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "slide");
        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setTargetPosition(0);
        wrist.setPosition(1);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.dpad_up) {
                wrist.setPosition(1);
            } else if (gamepad1.dpad_down) {
                wrist.setPosition(0.2);
            }
            if (gamepad1.a) {
                linearSlide.setTargetPosition(507);
            } else if (gamepad1.x) {
                linearSlide.setTargetPosition(6268);
            } else if (gamepad1.y) {
                linearSlide.setTargetPosition(9623);
            } else if (gamepad1.b) {
                linearSlide.setTargetPosition(13042);
            }
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(1);

//                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("Servo Position: ", pixel.getPosition());
                telemetry.addData("pos", drive.getWheelPositions());
                telemetry.addData("Velocities", drive.getWheelVelocities());
                telemetry.update();

        }
    }
}
