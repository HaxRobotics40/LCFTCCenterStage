package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name="Robot: Auto Drive To Red Line", group="Robot")
public class RobotAutoDriveToRedLine_Linear extends LinearOpMode {

    SampleMecanumDrive drive;
    private NormalizedColorSensor colorSensor;
    Trajectory trajectory;


    static final double RED_THRESHOLD = 0.4; // Adjust this threshold as needed
    static final double APPROACH_SPEED = 0.225;


    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        colorSensor.setGain(35);

        while (opModeInInit()) {
            telemetry.addData("Status", "Ready to drive to red line.");
            getRedValue();
            telemetry.addLine(drive.getWheelVelocities().toString());
        }

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(0.25)
                .build();



        while (opModeIsActive() && (getRedValue() < RED_THRESHOLD)) {
            telemetry.addLine(drive.getWheelVelocities().toString());
            drive.followTrajectory(myTrajectory);

        }
        telemetry.addLine("Rec");
//        drive.setMotorPowers(-0.1,-0.1,-0.1,-0.1);
//        sleep(800);
        drive.setMotorPowers(0,0,0,0);
    }

    double getRedValue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Red Value (0 to 1)", "%4.2f", colors.red);
        telemetry.update();
        return colors.red;
    }
}
