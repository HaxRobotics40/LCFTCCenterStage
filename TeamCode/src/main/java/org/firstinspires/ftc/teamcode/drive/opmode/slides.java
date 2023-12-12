package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.function.IntSupplier;
public class slides {
    DcMotor armMotor;
    // positions of arm
    private final ArrayList<Integer> levels;
    // tolerance for encoder values
    private int tolerance = 25;
    // most recent target level
    private int targetLevel = 0;

    public slides (@NonNull HardwareMap hw, String motorName) {
        armMotor = hw.get(DcMotor.class, motorName);
        // startup position is 0
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // make sure it can hold itself up
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        levels = new ArrayList<>();
    }

    // Adds new level with associated encoder position
    // returns object to enable method chaining
    public slides addLevel(int encoderPos) {
        levels.add(encoderPos);
        return this;
    }

    // erase all levels
    public void clearLevels() {
        levels.clear();
    }

    // used for joystick control, manually sets power
    public void setPower(double power) {
        if (power == 0 && targetLevel != -1) {
            return;
        }
        // make sure we don't go higher than the highest level
        if (armMotor.getCurrentPosition() - levels.get(levels.size() - 1) >= tolerance) {

            goTo(levels.size() - 1);
            return;
        }
        targetLevel = -1;
        armMotor.setPower(power);
    }

    // start targeting new level
    public void goTo(int level) {
        // if command is given when we are within tolerance, just stop the motor
        if (atLevel(level)) {
            armMotor.setPower(0);
            return;
        }
        if (targetLevel == level) {
            return;
        }
        targetLevel = level;
        double direction = Math.signum((double) levels.get(targetLevel) - armMotor.getCurrentPosition());
        armMotor.setPower(direction);

    }

    public void goTo(@NonNull IntSupplier supplier) {
        goTo(supplier.getAsInt());
    }

    // check if arm is at target, stopping it if it is
    public void update() {
        if (targetLevel == -1) {
            return;
        }
        if (atLevel(targetLevel)) {
            armMotor.setPower(0);
        }

    }

    // whether we are withing the tolerance for a given level
    public boolean atLevel(int level) {
        return Math.abs(armMotor.getCurrentPosition() - levels.get(level)) < tolerance;
    }

    public void setTolerance(int newTolerance) {
        tolerance = newTolerance;
    }


}