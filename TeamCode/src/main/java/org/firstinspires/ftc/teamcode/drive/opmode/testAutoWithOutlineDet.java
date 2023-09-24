package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;


@Autonomous
public class testAutoWithOutlineDet extends LinearOpMode {

    testEOCVpipeline detector;

    @Override
    public void runOpMode() throws InterruptedException {

        Thread telemetryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                    outputTelemetry();
                    try {
                        Thread.sleep(10); // Introducing a small delay to prevent excessive updates
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        });

        waitForStart();

        telemetryThread.start(); // Starting telemetry thread

        if (opModeIsActive()) {
            while (opModeIsActive()) {
//               opModeLoop();
            }
        }

        telemetryThread.interrupt(); // Make sure to interrupt the telemetry thread when opMode is no longer active
    }

    private void opModeLoop() {

    }


    private void outputTelemetry() {
        // TODO: Also output to .log file.
        telemetry.addLine("---------Piece Location----------");
        telemetry.addLine(Integer.toString(detector.pieceLocation()));
    }
}