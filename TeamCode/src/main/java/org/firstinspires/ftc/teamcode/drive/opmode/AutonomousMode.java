package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class AutonomousMode extends OpMode {
    //    Intake intake;
//    LinearSlide linearSlide;
    WebcamName webcamName;
//    Intake intake;
    OpenCvCamera webcam;
    SleeveDetector detector = new SleeveDetector(0.0325755, 162.051861606,162.051861606, 360, 360);
    double TheLocation;
    SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(36, 62, Math.toRadians(270));
    TrajectorySequence trajSeq;
    DcMotorEx FE;
    DcMotorEx LE;
    DcMotorEx RE;
    @Override
    public void init() {
        initRobot();
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    boolean stop = false;
                    while (!stop) {
                        if (detector.getLocationInt() != 4) {
                            TheLocation = detector.getLocationInt() - 2;
                            telemetry.addData("Location", detector.getLocation());
                            stop = true;
                        }
                    }
                    webcam.closeCameraDevice();
                    // Why do the old auto modes have a mix of both closeCameraDevice and stopStreaming?
                    // I feel liek this might crash. (I mean, it will, into the signal, but software wise.) (I don't trust my while loops) Please test
                })
                .waitSeconds(4)
                .splineTo(new Vector2d((36+(TheLocation*(-24))), 30), Math.toRadians(TheLocation*90+90))
                .build();
//        telemetry.addData("Color", detector.getLocation());
    }

    @Override
    public void start(){
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()  {
            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("onError", true);
                webcam.closeCameraDevice();
            }
        });
        // set opencv pipeline
        webcam.setPipeline(detector);
        drive.followTrajectorySequence(trajSeq);
        // it only returns START now for some reason
        // pattern - it'll first say cannot close camera when the camera hasnt been opened, and THEN itll dc the next time or dc immediately
        // do NOT put stop streaming because its not "streaming" somewhere, its just OPEN. its not ON TWITCH (metaphorically) DO NOT PUT IT HER EIT CRASHES EVERYTHING
    }

    @Override
    public void loop() {
        drive.update();
    }

    public void initRobot() {
//
//        linearSlide = new LinearSlide(hardwareMap, "linearSlideMotor");
//        linearSlide.addLevel(0).addLevel(94).addLevel(725).addLevel(1192).addLevel(1660);

//        intake = new Intake(hardwareMap, "intakeServo", "leftServo", "rightServo");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        FE = hardwareMap.get(DcMotorEx.class, "FE");
        LE = hardwareMap.get(DcMotorEx.class, "LE");
        RE = hardwareMap.get(DcMotorEx.class, "RE");
        FE.setDirection(DcMotorSimple.Direction.REVERSE);
        LE.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        intake = new Intake(hardwareMap, "intakeServo", "leftServo", "rightServo");
//        intake.Grab();

    }
}