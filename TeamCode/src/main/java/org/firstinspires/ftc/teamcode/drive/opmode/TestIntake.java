package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class TestIntake extends OpMode {

    private  Servo left;
    private  Servo right;
    private  DcMotor motor;
    boolean ap;
    boolean bp;
    public void init()  {
         left= hardwareMap.get(Servo.class, "left");
         right= hardwareMap.get(Servo.class, "right");
          motor = hardwareMap.get(DcMotor.class, "motor");
       boolean ap;
        ap=false;
         boolean bp;
        bp=false;
    }
    @Override
    public void loop() {
        if (gamepad1.a && !ap) {
            //motor.setPower(0.5);
            left.setPosition(left.getPosition() + 0.05);
            right.setPosition(right.getPosition() - 0.05);
            ap = true;
        } else if (!gamepad1.a) {
            ap = false;
        }
        if (gamepad1.b && !bp) {
            //motor.setPower(-0.5);
            left.setPosition(left.getPosition() - 0.05);
            right.setPosition(right.getPosition() + 0.05);
            bp = true;
        } else if (!gamepad1.b) {
            bp = false;
        }
    }
}

