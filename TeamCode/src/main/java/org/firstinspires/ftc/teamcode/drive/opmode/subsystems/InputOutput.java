package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.function.IntSupplier;
public class InputOutput {
    DcMotor pivot;
    DcMotor slide;
    Servo clawL;
    Servo clawR;
    Servo hook;
    Servo wrist;
    // positions of arm
    private final ArrayList<Integer> levels;
    // tolerance for encoder values
    private int tolerance = 25;
    // most recent target level
    private int targetLevel = 0;
    private int degAngle = 0;
    private final double ticksPerDeg = 3.9581;
//    private int lastLevel;
    private final double maxPowerSlide;
    private final double maxPowerPivot;

    public InputOutput(@NonNull HardwareMap hw, boolean autoFillLevels, double maxPowerSlide, double maxPowerPivot) {
        pivot = hw.get(DcMotor.class, "pivot");
        // startup position is 0
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // make sure it can hold itself up
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hw.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        clawL = hw.get(Servo.class, "clawL");
        clawR = hw.get(Servo.class, "clawR");
        wrist = hw.get(Servo.class, "wrist");

        levels = new ArrayList<>();
        
        if (autoFillLevels) {
            fillLevels();
        }
        this.maxPowerSlide = maxPowerSlide;
        this.maxPowerPivot = maxPowerPivot;
    }

    // Adds new level with associated encoder position
    // returns object to enable method chaining
    public InputOutput addLevel(int encoderPos) {
        levels.add(encoderPos);
        return this;
    }
    public void fillLevels() {
        this.addLevel(507).addLevel(5234).addLevel(10468).addLevel(13042);
    }
    // erase all levels
    public void clearLevels() {
        levels.clear();
    }

    // internal stuff, faster to use levels
    public void setPower(double power) {
        if (power == 0 && targetLevel != -1) {
            return;
        }
        // make sure we don't go higher than the highest level
        if (slide.getCurrentPosition() - levels.get(levels.size() - 1) >= tolerance) {

            goTo(levels.size() - 1);
            return;
        }
        targetLevel = -1;
        slide.setPower(power);
    }

    // start targeting new level
    public void goTo(int level) {
        // if command is given when we are within tolerance, just stop the motor
        if (atLevel(level) || level < -1 || level > levels.size()) {
            slide.setPower(0);
            return;
        }
        if (targetLevel == level) {
            return;
        }
        targetLevel = level;
        double direction = maxPowerSlide * Math.signum((double) levels.get(targetLevel) - slide.getCurrentPosition());
        slide.setPower(direction);

    }
    public void goTo(@NonNull IntSupplier supplier) {
        goTo(supplier.getAsInt());
    }

    private void setAngle(int deg) { // can change if 0 ticks isn't 0 deg actually
        double posDouble = deg * ticksPerDeg;
        int posInt = (int) posDouble;
        degAngle = deg;
        pivot.setTargetPosition(posInt);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(maxPowerPivot * Math.signum((double) posInt - pivot.getCurrentPosition()));

    }
    public void ground() {
        setAngle(0);
        wrist.setPosition(.73);
    }
    public void extendo() {
        setAngle(0);
        wrist.setPosition(.73);
        goTo(levels.size());
    }
    public void over() {
        setAngle(135);
        wrist.setPosition(1); // idk if we need this? are our slides long enough?
        // TODO: det if this is necessary; if so find deg
    }
    public void board() { // 60 deg
        setAngle(126);
        wrist.setPosition(.73);
    }

    public void grab() {
        clawL.setPosition(.65);
        clawR.setPosition(1);
    }

    public void release() {
        clawL.setPosition(0.9);
        clawR.setPosition(.85);
    }


    // check if arm is at target, stopping it if it is
    public void update() {
        if (targetLevel == -1) {
            return;
        }
        if (atLevel(targetLevel)) {
            slide.setPower(0);
        }

    }

    // whether we are withing the tolerance for a given level
    public boolean atLevel(int level) {
        return Math.abs(slide.getCurrentPosition() - levels.get(level)) < tolerance;
    }
    public int getLevel() {
        return targetLevel;
    }
    public int getAngle() {
        return degAngle;
    }
    public int getSlidePos() { return slide.getCurrentPosition(); }
    public int getPivotPos() { return pivot.getCurrentPosition(); }
    public double getRightPos() { return clawR.getPosition(); }
    public double getLeftPos() { return clawL.getPosition(); }
    public void setTolerance(int newTolerance) {
        tolerance = newTolerance;
    }


}