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
    boolean xPushed = false;
    boolean intakeUp = true;

    public void init()  {
         left= hardwareMap.get(Servo.class, "left");
         right= hardwareMap.get(Servo.class, "right");
          motor = hardwareMap.get(DcMotor.class, "motor");
//          left.setPosition(1);
//          right.setPosition(0);
       boolean ap;
        ap=false;
         boolean bp;
        bp=false;
    }
    @Override
    public void loop() {
        teleLogging(String.valueOf(left.getPosition())+"   " + String.valueOf(right.getPosition()));
        if (gamepad1.a && !ap) {
            //motor.setPower(0.5);
            left.setPosition(left.getPosition() + 0.01);
            right.setPosition(right.getPosition() - 0.01);
            ap = true;
            teleLogging("AAAAAA");
        } else if (!gamepad1.a) {
            ap = false;
        }
        if (gamepad1.b && !bp) {
            //motor.setPower(-0.5);
            left.setPosition(left.getPosition() - 0.01);
            right.setPosition(right.getPosition() + 0.01);
            bp = true;
            teleLogging("BBBBBB");
        } else if (!gamepad1.b) {
            bp = false;
        }
        motor.setPower(-gamepad1.right_trigger);
        teleLogging(String.valueOf(motor.getPower()));
        telemetry.update();
//        if (gamepad1.x && !xPushed) {
//            xPushed = true;
//            if(intakeUp){
////                left.setPosition();
//            } else {
//                //intake down
//            }
//            intakeUp = !intakeUp;
//
//        }

    }





    private void teleLogging(String s) {
        telemetry.addLine(s);
    }
}

