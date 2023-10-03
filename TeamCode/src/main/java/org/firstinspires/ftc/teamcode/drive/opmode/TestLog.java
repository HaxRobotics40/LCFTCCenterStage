package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.util.List;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;


@Autonomous
public class TestLog extends LinearOpMode {
    Logger logger;

    @Override
    public void runOpMode() throws InterruptedException {
//        initLogging();
        //drive = new SampleMecanumDrive(hardwareMap);
        // TODO: Fix Drive Constants physical measurements!!!
//        TODO: Move Rever
//         se to here.

//        Thread telemetryThread = new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
//                    outputTelemetry();
//                    try {
//                        Thread.sleep(10); // Introducing a small delay to prevent excessive updates
//                    } catch (InterruptedException e) {
//                        Thread.currentThread().interrupt();
//                    }
//                }
//            }
//        });

        waitForStart();

//        telemetryThread.start(); // Starting telemetry thread

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.setAutoClear(false);
               telemetry.addLine(String.valueOf(Math.random()*1000));
            }
        }

//        telemetryThread.interrupt(); // Make sure to interrupt the telemetry thread when opMode is no longer active
    }

    private void opModeLoop() {

    }

//    private void initLogging() {
//        Logger logger = Logger.getLogger("MyLogger");
//        // Set up FileHandler
//        FileHandler fileHandler = null;
//        try {
//            fileHandler = new FileHandler("/sdcard/FIRST/data/mylog.txt");
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//
//        // Specify the log format
//        SimpleFormatter formatter = new SimpleFormatter();
//        fileHandler.setFormatter(formatter);
//
//        ConsoleHandler consoleHandler = new ConsoleHandler();
//        SimpleFormatter consoleFormatter = new SimpleFormatter();
//        consoleHandler.setFormatter(consoleFormatter);
//
//        // Add both handlers to the logger
//        logger.addHandler(fileHandler);
//        logger.addHandler(consoleHandler);
//
//        // Set the logger level (optional, default is Level.INFO)
//        logger.setLevel(Level.INFO);
//    }



//    private void outputTelemetry() {
//        teleLogging("hello world");
//    }
//
//    private void teleLogging(String s) {
//        telemetry.addLine(s);
////        logger.info(s);
//    }
}
