package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class TestOutake extends OpMode {


    private  DcMotor motorSpin;
    private  DcMotor motorLift;


    public void init()  {
        motorSpin = hardwareMap.get(DcMotor.class, "motors");
        motorLift = hardwareMap.get(DcMotor.class, "motorl");
    }
    @Override
    public void loop() {
        motorLift.setPower(gamepad1.right_trigger);
        motorLift.setPower(-gamepad1.left_trigger);
    }





    private void teleLogging(String s) {

        telemetry.addLine(s);
    }
}

