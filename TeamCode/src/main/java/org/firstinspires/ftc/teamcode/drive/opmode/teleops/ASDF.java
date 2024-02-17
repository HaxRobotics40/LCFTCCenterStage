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
 *
 * --------------------------------------------------------------------------
 *<p>
 * DRIVER CONTROLS
 * <p>
 * Left Stick Up/Down/Left/Right      \\              Forward/Backward/Strafe<p>
 * Right Stick Left/Right      \\                     Turning<p>
 * Back    \\                                         Spin to face board<p>
 * Y                          \\                      Reset IMU - Turn to face Alliance Station before doing this<p>
 *<p>
 * Right Trigger & Left Trigger Together     \\       Launch Drone<p>
 *<p>
 * Press Start Once                          \\       Move Hook servo 180 degrees<p>
 * D-Pad Up                                    \\     Spin winch motor<p>
 * D-Pad Down                                   \\    Spin winch motor other way<p>
 *<p>
 * --------------------------------------------------------------------------
 *<p>
 * OPERATOR CONTROLS
 *<p>
 * Left Bumper                                 \\     Grab<p>
 * Right Bumper                              \\       Release<p>
 *<p>
 * X                                       \\         Pivot to Ground<p>
 * Y                                     \\           Pivot to go Under Trusses<p>
 * B                                   \\             Pivot to Board Angle<p>
 * A                                       \\         Pivot to Back Position<p>
 *<p>
 * D-Pad Left                                \\       Fold Wrist<p>
 * D-Pad Right                                 \\     Extend Wrist<p>
 *<p>
 * Press Right Trigger Once                      \\   Move Slide Up<p>
 * Press Left Trigger Once                         \\ Move Slide Down<p>
 *<p>
 * --------------------------------------------------------------------------
 */
@TeleOp(group = "drive")
@Config
public class ASDF extends LinearOpMode {
    FtcDashboard dashboard;
    boolean runningPID;
    boolean isDepressedUp = false;
    boolean isDepressedDown = false;
    boolean isDepressedStart= false;
    boolean wasUpPressed;
    boolean wasDownPressed;
    int hookStatus = 0;
    boolean wasStartPressed;
    public static double Kp = 0.6;
    public static double Ki = 0;
    public static double Kd = 0.048;

    public static double kP = 0.00085;
    public static double kI = 0.00001;
    public static double kD = 0;
    public static double kCos = -0.00005;
    double botHeading;
    double error;
    double tolerance = 3;
    double integralSum=0;
    double lastError = 0;
    double derivative;
    double targetAngle = -90;
    double rx;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        InputOutput arm = new InputOutput(hardwareMap, true, .5, .1);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        Servo droneRelease = hardwareMap.get(Servo.class, "droneRelease");
        DcMotor winch = hardwareMap.get(DcMotor.class, "winch");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo hook = hardwareMap.get(Servo.class, "hook");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(0);

        arm.setup();
        arm.clearLevels();
        arm.addLevel(0).addLevel(880).addLevel(1618);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        droneRelease.setPosition(1);
        hook.setPosition(0.875);

//        while (opModeInInit()) {
//            if (gamepad1.b) {
//                targetAngle = 90;
//            } else if (gamepad1.x) {
//                targetAngle = -90;
//            }
//        }
        waitForStart();

        while (!isStopRequested()) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (gamepad1.back) {
                runningPID = true;
            }
            if (!runningPID) {
                rx = -gamepad1.right_stick_x / 2;
            } else {
                if (Math.abs(botHeading - Math.toRadians(targetAngle)) < tolerance) {
                    runningPID = false;
                } else {
                    error = Math.toRadians(targetAngle) - botHeading;
                    integralSum = integralSum + (error * timer.seconds());
                    derivative = (error - lastError) / timer.seconds();

                    rx = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                    lastError = error;

                    timer.reset();
                }
            }
            if (gamepad1.y) {
                imu.resetYaw();
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            rx
                    )
            );
            buttonPressedDown();
            buttonPressedUp();
            buttonPressedStart();

            if (gamepad2.dpad_left) {
                arm.wristIn();
            } else if (gamepad2.dpad_right) {
                arm.wristOut();
            }


            if (gamepad2.left_bumper) {
                arm.grab();
            }
            if (gamepad2.right_bumper) {
                arm.release();
            }


            if (gamepad2.x) {
                arm.ground();
            } else if (gamepad2.y) {
                arm.out();
            } else if (gamepad2.b) {
                arm.frontBoard();
            } else if (gamepad2.a) {
                arm.board();
            }


            if (wasStartPressed) {
                hook.setPosition(0);
            }
            if (gamepad1.b) {
                wrist.setPosition(0.45);
                arm.addLevel(1000);
                arm.goTo(3);
                arm.setAngle(4);
            }

            if (gamepad1.dpad_up) {
                winch.setPower(1);
            } else if (gamepad1.dpad_down) {
                winch.setPower(-1);
            } else {
                winch.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1) {
                droneRelease.setPosition(0);
            }


            // preset positions: requires tuning/position determing and making sure directions are right

            if (wasUpPressed) {
                if (arm.getLevel() < 2) {
                    arm.goTo(arm.getLevel() + 1);
                    wasUpPressed = false;
                }
            } else if (wasDownPressed) {
                if (arm.getLevel() > 0) {
                    arm.goTo(arm.getLevel() - 1);
                    wasDownPressed = false;
                }
            }
            arm.getNewPIDF(kP, kI, kD, kCos);

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
            telemetry.addData("Slide tick", arm.getSlidePos());
//            telemetry.addData("Wrist Position", wrist.getPosition());
//            telemetry.addData()
            telemetry.addData("Wrist Pos", wrist.getPosition());
            telemetry.addData("Claw L & R Pos", arm.getLeftPos() + " " + arm.getRightPos());
            telemetry.update();

        }
    }
    public void buttonPressedUp() {
        if (gamepad2.right_trigger > 0.5 && !isDepressedUp) {
            isDepressedUp = true;
            wasUpPressed = false;
        }
        if (gamepad2.right_trigger < 0.5 && isDepressedUp) {
            isDepressedUp = false;
            wasUpPressed = true;
        }
    }
    public void buttonPressedDown() {
        if (gamepad2.left_trigger > 0.5 && !isDepressedDown) {
            isDepressedDown = true;
            wasDownPressed = false;
        }
        if (gamepad2.left_trigger < 0.5 && isDepressedDown) {
            isDepressedDown = false;
            wasDownPressed = true;
        }
    }
    public void buttonPressedStart() {
        if (gamepad1.start && !isDepressedStart) {
            isDepressedStart = true;
            wasStartPressed = false;
        }
        if (!gamepad1.start && isDepressedStart) {
            isDepressedStart = false;
            wasStartPressed = true;
        }
    }
}
