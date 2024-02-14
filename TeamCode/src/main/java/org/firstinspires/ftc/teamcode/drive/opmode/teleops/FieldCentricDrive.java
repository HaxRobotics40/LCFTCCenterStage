package org.firstinspires.ftc.teamcode.drive.opmode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
@TeleOp
@Config
public class FieldCentricDrive extends LinearOpMode {
    boolean isDepressedUp = false;
    FtcDashboard dashboard;
    boolean isDepressedDown = false;
    boolean isDepressedStart = false;
    boolean wasStartPressed;
    boolean wasUpPressed;
    boolean wasDownPressed;
    boolean runningPID;
    public static double Kp = 0.6;
    public static double Ki = 0;
    public static double Kd = 0.048;
    double tolerance = Math.toRadians(3);
    double error;
    double integralSum=0;
    double lastError = 0;
    double derivative;
    double botHeading;
    int hookStatus;
    double rx;
    double targetAngle = 90;
    //Sets value of Kp, Ki and Kd for PID controller
    public void setTunings(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
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
    @Override
    public void runOpMode() throws InterruptedException {
        InputOutput arm = new InputOutput(hardwareMap, true, .25, .15);
        // Declare our motors
        // Make sure your ID's match your configuration
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");

        DcMotor winch = hardwareMap.dcMotor.get("winch");
        Servo droneRelease = hardwareMap.servo.get("droneRelease");
        Servo hook = hardwareMap.servo.get("hook");
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //CHANGE THESE ONCE ORIENTATION IS KNOW
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setup();
        while (opModeInInit()) {
            if (gamepad1.b) {
                targetAngle = 90;
            } else if (gamepad1.x) {
                targetAngle = -90;
            }
        }
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference..


            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if(gamepad1.back) {
                runningPID = true;
            }
            //PID controller for heading
            if (runningPID) {
                if (Math.abs(botHeading - Math.toRadians(targetAngle)) < tolerance){
                    runningPID =  false;
                }
                else {
                    error = Math.toRadians(targetAngle)-botHeading;
                    integralSum = integralSum + (error * timer.seconds());
                    derivative = (error - lastError)/timer.seconds();

                    rx = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                    lastError = error;

                    timer.reset();
                }
            }

            // Rotate the movement direction counter to the bots rotation
            double fieldX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double fieldY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            fieldX = fieldX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rx), 1);
            double frontLeftPower = (fieldY + fieldX + rx);
            double backLeftPower = (fieldY - fieldX + rx) ;
            double frontRightPower = (fieldY - fieldX - rx) ;
            double backRightPower = (fieldY + fieldX - rx) ;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);



            buttonPressedDown();
            buttonPressedUp();
            buttonPressedStart();

            if (gamepad2.dpad_left) {
                arm.wristIn();
            } else if (gamepad2.dpad_right) {
                arm.wristOut();
            }

            if (gamepad1.y) {
                imu.resetYaw();
            }

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



            if (gamepad2.dpad_right) {
                arm.goTo(0);
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

            if (gamepad2.left_bumper) {
                arm.grab();
            }
            if (gamepad2.right_bumper) {
                arm.release();
            }

            if (wasStartPressed & hookStatus == 1) {
                hook.setPosition(0);
            } else if (wasStartPressed & hookStatus == 0) {
                hook.setPosition(1);
            }

            if (gamepad1.dpad_up) {
                winch.setPower(1);
            } else
                winch.setPower(0);
        }

        if (gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1) {
            droneRelease.setPosition(0);
        }
            arm.update();
            telemetry.addData("botheading", botHeading);
            telemetry.addData("rx", rx);
            telemetry.update();


    }
}
