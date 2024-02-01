package org.firstinspires.ftc.teamcode.drive.opmode.teleops;

//import android.renderscript.ScriptGroup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
public class ASDF extends LinearOpMode {
    FtcDashboard dashboard;
    boolean runningPID;
    boolean isDepressedUp = false;
    boolean isDepressedDown = false;
    boolean isDepressedBack = false;
    boolean wasUpPressed;
    boolean wasDownPressed;
    Telemetry tm;
    boolean wasBackPressed;
    public static double Kp = 0.6;
    public static double Ki = 0;
    public static double Kd = 0.048;

    public static double kP = 0.0004625;
    public static double kI = 0.000001;
    public static double kD = 0;
    public static double kCos = -0.00003;
    ElapsedTime timer = new ElapsedTime();
    double botHeading;
    double error;
    double tolerance = 3;
    double integralSum=0;
    double lastError = 0;
    double derivative;
    double rx;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        InputOutput arm = new InputOutput(hardwareMap, true, .25, .1);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        Servo droneRelease = hardwareMap.get(Servo.class, "droneRelease");
        DcMotor winch = hardwareMap.get(DcMotor.class, "winch");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo hook = hardwareMap.get(Servo.class, "hook");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
//        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");
        Servo clawL = hardwareMap.get(Servo.class, "clawL");
        Servo clawR = hardwareMap.get(Servo.class, "clawR");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
 //finql
        arm.grab();
        wrist.setPosition(0);
//        pivot.setTargetPosition(0);
//        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setup();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        droneRelease.setPosition(1);
        waitForStart();

        while (!isStopRequested()) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (gamepad1.back) {
                runningPID = true;
            }
            if (!runningPID) {
                rx = -gamepad1.right_stick_x;
            } else {
                if (Math.abs(botHeading - Math.toRadians(-90)) < tolerance){
                    runningPID =  false;
                }
                else {
                    error = Math.toRadians(-90)-botHeading;
                    integralSum = integralSum + (error * timer.seconds());
                    derivative = (error - lastError)/timer.seconds();

                    rx = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                    lastError = error;

                    timer.reset();
                }
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            rx
                    )
            );

            if (gamepad2.dpad_left) {
                wrist.setPosition(0);
            } else if (gamepad2.dpad_right) {
                wrist.setPosition(.95);
            }
            // analog up/down for pivot & arm & wrist


            if (gamepad2.left_bumper) {
//                arm.grabLeft();
                arm.grab(); }
            if (gamepad2.right_bumper) {
                arm.release();
            }

//
//

            if (gamepad2.x) {
                arm.ground();
            } else if (gamepad2.y) {
                arm.out();
            } else if (gamepad2.b) {
                arm.frontBoard();
            } else if (gamepad2.a) {
                arm.board();
            }







            if (gamepad2.a) {
                hook.setPosition(0);
            }
            else if (gamepad2.dpad_left) {
                hook.setPosition(1);
            }

            if (gamepad2.right_trigger > .5) {
                winch.setPower(1);
            } else if (gamepad2.left_trigger > .5) {
                winch.setPower(-1);
            } else {
                winch.setPower(0);
            }

            if (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1) {
                droneRelease.setPosition(0);
            } else if (gamepad2.start) {
                droneRelease.setPosition(1);
            }



            // preset positions: requires tuning/position determing and making sure directions are right

            buttonPressedDown();
            buttonPressedUp();
            if (wasUpPressed) {
                if (arm.getLevel() < 3) {
                    arm.goTo(arm.getLevel() + 1);
                    wasUpPressed = false; }
            } else if (wasDownPressed) {
                if (arm.getLevel() > 0) {
                    arm.goTo(arm.getLevel() - 1);
                    wasDownPressed = false; }
            }
                arm.getNewPIDF(kP, kI, kD, kCos);
//
//
//            if (gamepad2.left_bumper) {
//                arm.release();
//            } else if (gamepad2.right_bumper) {
//                arm.grab();
//            }
//
//
//
            drive.update();
            arm.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("pos", drive.getWheelPositions());
//            telemetry.addData("Velocities", drive.getWheelVelocities());
//            telemetry.addData("Elevator Position", slide.getCurrentPosition());
            telemetry.addData("Arm Position", arm.getPivotPos());
            telemetry.addData("Desired Arm Position", arm.getTargetPivotPos());
            telemetry.addData("Motor Power", arm.getPivotPower());
//            telemetry.addData("Wrist Position", wrist.getPosition());
//            telemetry.addData()
            telemetry.addData("Wrist Pos", wrist.getPosition());
            telemetry.addData("Claw L & R Pos", arm.getLeftPos() + " "  + arm.getRightPos());
            telemetry.update();

        }
    }
    public void buttonPressedUp() {
        if (gamepad2.dpad_up && !isDepressedUp) {
            isDepressedUp = true;
            wasUpPressed = false;
        }
        if (!gamepad2.dpad_up && isDepressedUp) {
            isDepressedUp = false;
            wasUpPressed = true;
        }
    }
    public void buttonPressedback() {
        if (gamepad2.back && !isDepressedBack) {
            isDepressedUp = true;
            wasUpPressed = false;
        }
        if (!gamepad2.back && isDepressedBack) {
            isDepressedUp = false;
            wasUpPressed = true;
        }
    }
    public void buttonPressedDown() {
        if (gamepad2.dpad_down && !isDepressedDown) {
            isDepressedDown = true;
            wasDownPressed = false;
        }
        if (!gamepad2.dpad_down && isDepressedDown) {
            isDepressedDown = false;
            wasDownPressed = true;
        }
    }
}
