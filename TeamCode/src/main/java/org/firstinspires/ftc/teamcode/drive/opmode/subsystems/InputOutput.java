package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.function.IntSupplier;
public class InputOutput {
    DcMotorEx pivot;
    DcMotor slide;
    Servo clawL;
    Servo clawR;
    Servo hook;
    Servo wrist;
    boolean ground;
    int targetPosition;
    // positions of arm
    private final ArrayList<Integer> levels;
    // tolerance for encoder values
    private int tolerance = 25;
    // most recent target level
    private int targetLevel = 0;
    private final int tolerancePivot = 8;
    private int degAngle = 0;
    int currentPos;
    private final double ticksPerDeg = 3.9581;
//    private int lastLevel;
    private final double maxPowerSlide;
    private final double maxPowerPivot;
    private final int[] levelsPivot = {500, 350, 270, 0}; // ground, outward, fboard, at rest.
    private final int[] anggleLevels = {90, 45, 60, 90};
    double targetAngle;
    private double  kP = .00085;
    public static double kI = 0.00001;
    public static double kD = 0;
    private double kCos = -0.00005;
//private double kP = 0;
//    private double kI = 0;
//    private double kD = 0;
//    private double kCos = 0;
    ElapsedTime timer;
    double integralSum;
    int lastError;
    public InputOutput(@NonNull HardwareMap hw, boolean autoFillLevels, double maxPowerSlide, double maxPowerPivot) {
        pivot = hw.get(DcMotorEx.class, "pivot");
        timer = new ElapsedTime();
        // startup position is 0
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // make sure it can hold itself up
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        slide = hw.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        clawL = hw.get(Servo.class, "clawL");
        clawR = hw.get(Servo.class, "clawR");
        wrist = hw.get(Servo.class, "wrist");
        slide.setDirection(DcMotor.Direction.REVERSE);

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
        this.addLevel(0).addLevel(638).addLevel(1276).addLevel(1914);
    }
    // erase all levels
    public void clearLevels() {
        levels.clear();
    }
    public void runFullPivotPower() { pivot.setPower(1);}
    public void getNewPIDF(double P, double I, double D, double F) {
        kP = P;
        kI = I;
        kD = D;
        kCos = F;
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

    public void setAngle(int levelPivot) { // can change if 0 ticks isn't 0 deg actually
        targetAngle = Math.toRadians(anggleLevels[levelPivot]);
        targetPosition = (levelsPivot[levelPivot]);
        if(levelPivot ==0) {
            ground = true;
        } else {
            ground = false;
        }
//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if (Math.abs(pivot.getCurrentPosition() - pivot.getTargetPosition()) > 4) {
//            pivot.setPower(maxPowerPivot * Math.signum((double) levelsPivot[levelPivot] - pivot.getCurrentPosition()));
//        } else {
//            pivot.setPower(0);
//        }
    }
    public void setup () {
        wrist.setPosition(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setTargetPosition(0);
        this.grab();
        this.rest();
    }
    public void ground() {
        setAngle(0);
        wrist.setPosition(1);
    }
    public void frontBoard() {
        setAngle(2);
        wrist.setPosition(1);
    }
    public void rest() {
        setAngle(3);
        goTo(0);
        if (atAngle()) { wrist.setPosition(0); }
    }
    public void board() { // 60 deg
        setAngle(3);
        wrist.setPosition(1);

    }
    public void out() {
        setAngle(1);
        goTo(0);
        if (atAngle()) { wrist.setPosition(0); }
    }
    public void grab() {
        clawL.setPosition(0);
        clawR.setPosition(.759);
    }

    public void release() {
        clawL.setPosition(.33);
        clawR.setPosition(.48);
    }

    public void grabLeft() { clawL.setPosition(0); }
    public void grabRight() { clawR.setPosition(.759); }
    public void releaseLeft() {clawL.setPosition(.33);}
    public void releaseRight() {clawR.setPosition(.48);}
    // check if arm is at target, stopping it if it is
    public void update() {

        if (atLevel(targetLevel)) {
            slide.setPower(0);
        }

//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if (Math.abs(pivot.getCurrentPosition() - pivot.getTargetPosition()) > 4) {
//            pivot.setPower(maxPowerPivot * Math.signum((double) targetPosition - pivot.getCurrentPosition()));
//        } else {
//            pivot.setPower(0);
//        }
        double output = kCos * Math.cos(targetAngle);

        currentPos = pivot.getCurrentPosition();
        int error = targetPosition - currentPos;

        double derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        output += (kP * error) + (kI * integralSum) + (kD * derivative);

        if (!ground) {
            pivot.setPower(output);
        } else if (ground && error > 20) {
            pivot.setPower(output);
        } else if (ground && error < 20) {
            pivot.setPower(0);
        }

        lastError = error;
        timer.reset();





        if (targetLevel == -1) {
            return;
        }
    }
    public boolean atAngle() {
        return Math.abs(pivot.getCurrentPosition() - targetPosition) < tolerancePivot;
    }

    // whether we are withing the tolerance for a given level
    public boolean atLevel(int level) {
        return Math.abs(slide.getCurrentPosition() - levels.get(level)) < tolerance;
    }
    public int getLevel() {
        return targetLevel;
    }
    public void setWrist(double position) { wrist.setPosition(position); }
    public int getAngle() {
        return degAngle;
    }
    public int getSlidePos() { return slide.getCurrentPosition(); }
    public int getPivotPos() { return currentPos; }
    public int getTargetPivotPos() { return targetPosition; }
    public double getPivotPower() { return pivot.getPower(); }

    public double getRightPos() { return clawR.getPosition(); }
    public double getLeftPos() { return clawL.getPosition(); }
    public void setTolerance(int newTolerance) {
        tolerance = newTolerance;
    }


}