package org.firstinspires.ftc.teamcode.drive.opmode.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "Robot: Align To Red/Blue Line", group = "Robot")
public class Color_Sensor extends LinearOpMode {

    SampleMecanumDrive drive;
    NormalizedColorSensor colorSensor;

    static final double LINE_THRESHOLD = 0.4; // Adjust this threshold as needed
    static final double APPROACH_SPEED = 0.225;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        colorSensor.setGain(35);

        waitForStart();

        while (opModeIsActive()) {
            // Check for both red and blue lines
            double redValue = getRedValue();
            double blueValue = getBlueValue();

            telemetry.addData("Red Value (0 to 1)", "%4.2f", redValue);
            telemetry.addData("Blue Value (0 to 1)", "%4.2f", blueValue);
            telemetry.update();

            if (redValue > LINE_THRESHOLD || blueValue > LINE_THRESHOLD) {
                // We found a line (either red or blue)
                drive.setMotorPowers(0, 0, 0, 0); // Stop the robot
                break; // Exit the loop
            } else {
                // Continue moving forward if no line is detected
                Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                        .forward(APPROACH_SPEED)
                        .build();

                drive.followTrajectory(myTrajectory);
            }
        }
    }

    double getRedValue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.red;
    }

    double getBlueValue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.blue;
    }
}
