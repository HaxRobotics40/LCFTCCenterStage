package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Robot: Auto Drive To Red Line", group="Robot")
public class RobotAutoDriveToRedLine_Linear extends LinearOpMode {

    SampleMecanumDrive drive;
    private NormalizedColorSensor colorSensor;


    static final double RED_THRESHOLD = 0.4; // Adjust this threshold as needed
    static final double APPROACH_SPEED = 0.15;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        colorSensor.setGain(100);

        while (opModeInInit()) {
            telemetry.addData("Status", "Ready to drive to red line.");
            getRedValue();
        }

        drive.setMotorPowers(APPROACH_SPEED,APPROACH_SPEED,APPROACH_SPEED,APPROACH_SPEED);

        while (opModeIsActive() && (getRedValue() < RED_THRESHOLD)) {
            sleep(5);
        }
        telemetry.addLine("Rec");
        drive.setMotorPowers(0,0,0,0);
    }

    double getRedValue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Red Value (0 to 1)", "%4.2f", colors.red);
        telemetry.update();
        return colors.red;
    }
}
