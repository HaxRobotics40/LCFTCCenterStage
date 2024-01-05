package org.firstinspires.ftc.teamcode.drive.opmode.teleops;

//import android.renderscript.ScriptGroup;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;

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
    boolean isDepressedUp = false;
    boolean isDepressedDown = false;
    boolean wasUpPressed;
    boolean wasDownPressed;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        InputOutput arm = new InputOutput(hardwareMap, true, .25, .15);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo hook = hardwareMap.get(Servo.class, "hook");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setTargetPosition(0);
        Servo clawL = hardwareMap.get(Servo.class, "clawL");
        Servo clawR = hardwareMap.get(Servo.class, "clawR");

        clawL.setPosition(.65);
        clawR.setPosition(1);
        wrist.setPosition(.73);


        waitForStart();

        while (!isStopRequested()) {
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );
            // analog up/down for pivot & arm & wrist
            if (gamepad1.dpad_left) {
                wrist.setPosition(wrist.getPosition()-.01);
            } else if (gamepad1.dpad_right) {
                wrist.setPosition(wrist.getPosition()+.01);
            }

            if (gamepad1.left_bumper) {
                clawL.setPosition(clawL.getPosition()+.002);
            } else if (gamepad1.left_trigger > .1) {
                clawL.setPosition(clawL.getPosition()-.002);
            }
            if (gamepad1.right_bumper) {
                clawR.setPosition(clawR.getPosition()+.002);
            } else if (gamepad1.right_trigger > .1) {
                clawR.setPosition(clawR.getPosition()-.002);
            }

            if (gamepad1.dpad_up) {
                slide.setTargetPosition(slide.getCurrentPosition()+5);
            } else if (gamepad1.dpad_down) {
                slide.setTargetPosition(slide.getCurrentPosition()-5);
            }


            if (gamepad1.x) {
                pivot.setTargetPosition(pivot.getCurrentPosition()-5);
            } else if (gamepad1.b) {
                pivot.setTargetPosition(pivot.getCurrentPosition()+5);
            }
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.2);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(0.1);




            // preset positions: requires tuning/position determing and making sure directions are right

//            buttonPressedDown();
//            buttonPressedUp();
//            if (wasUpPressed) {
//                arm.goTo(arm.getLevel() + 1);
//                wasUpPressed = false;
//            } else if (wasDownPressed) {
//                arm.goTo(arm.getLevel() - 1);
//                wasDownPressed = false;
//            }
//
//            if (gamepad1.x) {
//                arm.ground();
//            } else if (gamepad1.y) {
//                arm.board();
//            } else if (gamepad1.b) {
//                arm.over();
//            }
//
//            if (gamepad1.left_bumper) {
//                arm.release();
//            } else if (gamepad1.right_bumper) {
//                arm.grab();
//            }



            drive.update();
            arm.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("pos", drive.getWheelPositions());
            telemetry.addData("Velocities", drive.getWheelVelocities());
            telemetry.addData("Elevator Position", arm.getSlidePos());
            telemetry.addData("Arm Position", arm.getPivotPos());
            telemetry.addData("Wrist Pos", wrist.getPosition());
            telemetry.addData("Claw L & R Pos", arm.getLeftPos() + " "  + arm.getRightPos());
            telemetry.update();

        }
    }
    public void buttonPressedUp() {
        if (gamepad1.dpad_up && !isDepressedUp) {
            isDepressedUp = true;
            wasUpPressed = false;
        }
        if (!gamepad1.dpad_up && isDepressedUp) {
            isDepressedUp = false;
            wasUpPressed = true;
        }
    }
    public void buttonPressedDown() {
        if (gamepad1.dpad_down && !isDepressedDown) {
            isDepressedDown = true;
            wasDownPressed = false;
        }
        if (!gamepad1.dpad_down && isDepressedDown) {
            isDepressedDown = false;
            wasDownPressed = true;
        }
    }
}
