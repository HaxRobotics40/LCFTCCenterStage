package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group="drive")
public class fcDrive extends LinearOpMode {
    SampleMecanumDrive drive;
    double rotate;
    double strafe;
    double forward;
    Pose2d pose;
    Pose2d startPose;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPose = new Pose2d(-36, -60, Math.toRadians(90)); // get from .json files later
        pose = startPose;
        waitForStart();

        while (!isStopRequested()) {
            BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            // Technically this is the default, however specifying it is clearer
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            // Without this, data retrieving from the IMU throws an exception
            imu.initialize(parameters);

            double botHeading = -imu.getAngularOrientation().firstAngle;


            rotate = gamepad1.right_stick_x;
            strafe =  gamepad1.left_stick_x * Math.cos(botHeading) - -gamepad1.left_stick_y * Math.sin(botHeading);
            forward = (-gamepad1.left_stick_y * Math.sin(botHeading)) + (gamepad1.left_stick_x * Math.cos(botHeading));


            double[] wheelSpeeds = {
                    // front left
                    forward + strafe + rotate,
                    // front right
                    forward - strafe - rotate,
                    // back left
                    forward - strafe + rotate,
                    // back right
                    forward + strafe - rotate
            };

//                normalize(wheelSpeeds);

            // set power
            drive.setMotorPowers(wheelSpeeds[0],wheelSpeeds[1],wheelSpeeds[2],wheelSpeeds[3]);


//
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            gamepad1.left_stick_y,
//                            gamepad1.left_stick_x,
//                            pose.getHeading() - gamepad1.right_stick_x));
//            pose = drive.getPoseEstimate();
//            BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//            // Technically this is the default, however specifying it is clearer
//            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//            // Without this, data retrieving from the IMU throws an exception
//            imu.initialize(parameters);
//
//            double botHeading = -imu.getAngularOrientation().firstAngle;
        }
    }

//
//    public void fcDrive(double forward, double strafe, double rotate) {
//
//        // Retrieve the IMU from the hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // Technically this is the default, however specifying it is clearer
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, data retrieving from the IMU throws an exception
//        imu.initialize(parameters);
//
//        double botHeading = -imu.getAngularOrientation().firstAngle;

//
//        rotate = gamepad1.right_stick_x;
//        strafe =  gamepad1.left_stick_x * Math.cos(botHeading) - -gamepad1.left_stick_y * Math.sin(botHeading);
//        forward = (-gamepad1.left_stick_y * Math.sin(botHeading)) + (gamepad1.left_stick_x * Math.cos(botHeading));

//
//        double[] wheelSpeeds = {
//                // front left
//                forward + strafe + rotate,
//                // front right
//                forward - strafe - rotate,
//                // back left
//                forward - strafe + rotate,
//                // back right
//                forward + strafe - rotate
//        };

        // normalize powers
        //normalize(wheelSpeeds);

//        // set power
//        frontLeft.setPower(wheelSpeeds[0]);
//        frontRight.setPower(wheelSpeeds[1]);
//        backLeft.setPower(wheelSpeeds[2]);
//        backRight.setPower(wheelSpeeds[3]);
//    }

//    public void Drive(double forward, double strafe, double rotate) {



        // normalize powers
        //normalize(wheelSpeeds);

        // set power
//        frontLeft.setPower(wheelSpeeds[0]);
//        frontRight.setPower(wheelSpeeds[1]);
//        backLeft.setPower(wheelSpeeds[2]);
//        backRight.setPower(wheelSpeeds[3]);
//    }


}

