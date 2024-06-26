//package org.firstinspires.ftc.teamcode.drive.opmode.autoModes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.InputOutput;
//import org.firstinspires.ftc.teamcode.drive.opmode.vision.testEOCVpipeline;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//
//@Autonomous(group = "comp", preselectTeleOp = "ASDF")
//@Config
//public class Far2Plus0FSM extends LinearOpMode {
//    VisionPortal.Builder vPortalBuilder;
//    VisionPortal vPortal;
//    ElapsedTime timer2;
//    ElapsedTime timer;
//    boolean startedDriving = false;
//    testEOCVpipeline detector = new testEOCVpipeline();
//    //    TODO: Use dead wheels
//    SampleMecanumDrive drive;
//    //TODO: Update Constants to be 100% accurate (ex. wheel radius)
//    IMU imu;
//    DistanceSensor sensorDistance;
//    InputOutput arm;
//    int status= 0;
//    int itemSector;
//    Pose2d startPose = new Pose2d(0,0,0);
//    double distForward = 0;
//    boolean isTimer2Started = false;
//    boolean overridden = false;
//    int isBlue = -1;
//    int parkSide = -1;
//    double output = 0.1;
//    public static double delayTime;
//    boolean isTimerStarted = false;
//    Pose2d distanceSensePose;
//    Trajectory driveToLine;
//    TrajectorySequence driveToBoard;
//    TrajectorySequence boardCorrection;
//    TrajectorySequence park;
//
//    public enum STATES {
//        INIT,
//        ALIGN_LINE,
//        SCORE_PURPLE,
//        DROP,
//        ALIGN_BOARD,
//        DELAY,
//        DISTANCE_SENSE,
//        SCORE_YELLOW,
//        DROP_YELLOW,
//        PARK,
//        STOP;
//    }
//    private STATES previousState = STATES.INIT;
//    private STATES currentState = STATES.INIT;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        if (opModeInInit()) {
//            arm = new InputOutput(hardwareMap, true, .5, .5);
//            Servo hook = hardwareMap.get(Servo.class, "hook");
//            vPortal = initVisionPortal();
//            arm.setup();
//            setupIMU();
//            setupDistanceSensor();
//            hook.setPosition(.875);
//            vPortal.setProcessorEnabled(detector, true);
//            status = 0;
//
//            drive = new SampleMecanumDrive(hardwareMap);
//            // sets up devices once & vportal
//            while (opModeInInit()) {
//                if (gamepad1.dpad_left) {
//                    parkSide = 1;
//                } else if (gamepad1.dpad_right) {
//                    parkSide = -1;
//                }
//
//                if (detector.getColor() == "RED" & !overridden) {
//                    isBlue = -1;
//                    startPose = new Pose2d(12,-66, Math.toRadians(270));
//                    drive.setPoseEstimate(startPose);
//                } else if (detector.getColor() == "BLUE" & !overridden) {
//                    isBlue = 1;
//                    startPose = new Pose2d(12,66, Math.toRadians(90));
//                    drive.setPoseEstimate(startPose);
//                }
//
//                if (gamepad1.b) {
//                    overridden = true;
//                    isBlue = 1;
//                    startPose = new Pose2d(12,66, Math.toRadians(90));
//                    drive.setPoseEstimate(startPose);
//                } else if (gamepad1.x) {
//                    isBlue = -1;
//                    startPose = new Pose2d(12,-66, Math.toRadians(270));
//                    drive.setPoseEstimate(startPose);
//                }
//                // controls - use the gamepad while init is running to change things like parkside
//                // wait until the "LOCATION" "COLOR" things say things until starting - all the camera processing happens here
//
//                if (detector.locationInt() != -1) {
//                    itemSector = detector.locationInt();
//                } else {
//                    itemSector = 2;
//                }
//                if (itemSector == 2) {
//                    driveToLine = drive.trajectoryBuilder(startPose)
//                            .lineToLinearHeading(new Pose2d(-36, isBlue*50, Math.toRadians((-isBlue*90)+((itemSector-1)*-33.4))))
//                            .build();
//                } else if (itemSector == 1) {
//                    driveToLine = drive.trajectoryBuilder(startPose)
//                            .lineToSplineHeading(new Pose2d(-36, isBlue * 45, Math.toRadians((-isBlue * 90) - 10)))
//                            .build();
//                } else if (itemSector == 0) {
//                    driveToLine = drive.trajectoryBuilder(startPose)
//                            .lineToLinearHeading(new Pose2d(-36, isBlue*50, Math.toRadians((-isBlue*90)+((itemSector-1)*-29.4))))
//                            .build();
//                }
//                if (itemSector == 0) {
//                    driveToBoard = drive.trajectorySequenceBuilder(driveToLine.end())
//                            .splineToLinearHeading(new Pose2d(-36, 60*isBlue, Math.toRadians(180)), Math.toRadians(0))
//                            .addDisplacementMarker(() -> {
//                                arm.wristIn();
//                            })
//                            .lineTo(new Vector2d(12, 60 * isBlue))
//                            .addDisplacementMarker(() -> {
//                                arm.frontBoard();
//                            })
//                            .splineToConstantHeading(new Vector2d(48, 20*isBlue), Math.toRadians(0))
//                            .build();
//                } else {
//
//                    if (itemSector == 2) {
//                        driveToLine = drive.trajectoryBuilder(startPose)
//                                .lineToLinearHeading(new Pose2d(-36, isBlue*50, Math.toRadians((-isBlue*90)+((itemSector-1)*-33.4))))
//                                .build();
//                    } else if (itemSector == 1) {
//                        driveToLine = drive.trajectoryBuilder(startPose)
//                                .lineToSplineHeading(new Pose2d(-36, isBlue * 45, Math.toRadians((-isBlue * 90) - 10)))
//                                .build();
//                    } else if (itemSector == 0) {
//                        driveToLine = drive.trajectoryBuilder(startPose)
//                                .lineToLinearHeading(new Pose2d(-36, isBlue*50, Math.toRadians((-isBlue*90)+((itemSector-1)*-29.4))))
//                                .build();
//                    }
//                    if (itemSector == 0) {
//                        driveToBoard = drive.trajectorySequenceBuilder(driveToLine.end())
//                                .splineToLinearHeading(new Pose2d(-36, 60*isBlue, Math.toRadians(180)), Math.toRadians(0))
//                                .addDisplacementMarker(() -> {
//                                    arm.wristIn();
//                                })
//                                .lineTo(new Vector2d(12, 60 * isBlue))
//                                .addDisplacementMarker(() -> {
//                                    arm.frontBoard();
//                                })
//                                .splineToConstantHeading(new Vector2d(48, 20*isBlue), Math.toRadians(0))
//                                .build();
//                    }
////                    else {
////                        driveToBoard = drive.trajectorySequenceBuilder(driveToLine.end())
////                                .splineToLinearHeading(new Pose2d(-36, 60*isBlue, Math.toRadians(180)), Math.toRadians(0))
////                                .addDisplacementMarker(() -> {
////                                    arm.wristIn();
////                                })
////                                .lineTo(new Vector2d(12, 60 * isBlue))
////                                .addDisplacementMarker(() -> {
////                                    arm.frontBoard();
////                                })
////                                .splineToConstantHeading(new Vector2d(48, (34+((itemSector-1)*6))*isBlue), Math.toRadians(0))
////                                .build();
////                    }        .lineTo(new Vector2d(12, 60 * isBlue))
////                            .addDisplacementMarker(() -> {
////                                arm.frontBoard();
////                            })
////                            .splineToConstantHeading(new Vector2d(48, (34+((itemSector-1)*6))*isBlue), Math.toRadians(0))
////                            .build();
////                }
//
//
//
//                // set up the pre-runnable auto paths
//                // driveToLine moves forward and angles the robot to drop on the purple pixel
//                // driveToBoard moves it from this position after the pixel dropping part to the board
//
//
//
//                telemetry.addData("Parking side", parkSide);
//                telemetry.addData("Location", detector.locationInt());
//                telemetry.addData("Color", detector.getColor());
//                telemetry.addData("Pose estimate", drive.getPoseEstimate().toString());
//                telemetry.update();
//            }
//        }
//
//        waitForStart();
//        if (opModeIsActive()) {
//            currentState = STATES.ALIGN_LINE; // when it starts, change the state to an active one
//            while (opModeIsActive()) {
//                opModeLoop(); // this possesses the FSM
//            }
//        }
//    }
//    private VisionPortal initVisionPortal() {
//        vPortalBuilder = new VisionPortal.Builder();
//        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
//        vPortalBuilder.addProcessor(detector);
//
//        return vPortalBuilder.build();
//    }
//    private void opModeLoop() {
//        switch(currentState) {
//            case INIT:
//                break;
//            case ALIGN_LINE:
//                if (previousState != currentState) {
//                    // everything in here will run once when the state switches
//
//                    drive.followTrajectoryAsync(driveToLine); // drives to the line to prepare to drop
//                    previousState = STATES.ALIGN_LINE;
//                } else if (!drive.isBusy()) {
//                    currentState = STATES.SCORE_PURPLE;
//                }
//                break;
//            case SCORE_PURPLE:
//                if (previousState != currentState) {
//                    // everything in here will run once when the state switches
//                    arm.ground(); // sets the target position of the arm to the ground level
//
//
//                    // this if statement makes it run arm.ground() in the while loop until its within enough ticks
//                    if (arm.atAngle()) {
//                        arm.releaseLeft();
//                        previousState = STATES.SCORE_PURPLE;
//                    }
//                } else { // this else statement isn't an elif because rr isn't running
//                    currentState = STATES.DROP;
//                }
//                break;
//            case DROP:
//                if (previousState!=currentState) {
//                    // isTimerStarted is a class variable set as false. when run the first time it
//                    if (!isTimerStarted) {
//                        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//                        timer.reset(); // starts timer
//                        isTimerStarted = true; // stops this block from running again
//                    }
//                    telemetry.addData("timer",timer.seconds());
//                    if (timer.seconds() > .25) {
//                        arm.out();
//                        if (arm.atAngle()) {
//                            previousState = STATES.DROP;
//                        }
//                    }
//                } else {
//                    currentState = STATES.DELAY;
//                }
//                break;
//            case DELAY:
//                if (previousState != currentState) {
//                    ElapsedTime delayTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//                    if (delayTimer.time() > delayTime) {
//                        previousState = STATES.DELAY;
//                    }
//                } else if (!drive.isBusy()) {
//                    currentState = STATES.ALIGN_BOARD;
//                }
//                break;
//            case ALIGN_BOARD:
//                if (previousState != currentState) {
//
//                    drive.followTrajectorySequenceAsync(driveToBoard);
//                    previousState = STATES.ALIGN_BOARD;
//                } else if (!drive.isBusy()) {
//                    currentState = STATES.DISTANCE_SENSE;
//                }
//                break;
//            case DISTANCE_SENSE:
//                if (previousState != currentState) {
//                    double wantedDistance = 1.25; // how far away you want the robot to go
//                    double thresholdDistanceInches = 0.1;
//
//                    distForward = sensorDistance.getDistance(DistanceUnit.INCH);
//
//                    /*
//                    checks if it's a valid value, returns the difference
//
//                     */
//                    if (sensorDistance.getDistance(DistanceUnit.INCH) != DistanceUnit.infinity && sensorDistance.getDistance(DistanceUnit.INCH) != 0) {
//                        distForward = sensorDistance.getDistance(DistanceUnit.INCH);
//                        if (Math.abs(distForward - wantedDistance) > thresholdDistanceInches) {
//                            output = wantedDistance - distForward;
//                        }
//                    } else {
//                        output = 0.1;
//                    }
//
//                    Pose2d lastPos = drive.getPoseEstimate();
//                    arm.wristOut();
//                    distanceSensePose = (new Pose2d(72 - (distForward + 16.52), lastPos.getY(), lastPos.getHeading()));
//                    boardCorrection = drive.trajectorySequenceBuilder(driveToBoard.end()) // TODO change this back
//                            .turn(Math.toRadians(180))
//                            .forward(-output)
//                            .build();
//                    drive.followTrajectorySequenceAsync(boardCorrection);
//                    startedDriving = true;
//                    // moves forward the distance and continues
//                    previousState = STATES.DISTANCE_SENSE;
//                } else if (startedDriving && !drive.isBusy()){
//                    currentState = STATES.SCORE_YELLOW;
//                }
//                break;
//            case SCORE_YELLOW:
//                if (previousState != currentState) {
//                     // moves the wrist (servo) and sets arm to scoring pose
//                    if (!isTimer2Started) {
//                        timer2 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//                        timer2.reset(); // starts timer
//                        isTimer2Started = true; // stops this block from running again
//                    }
//                    telemetry.addData("Timer", timer2.seconds());
//                    if (timer2.time() > 0.25) {
//                        arm.release();
//                    }
//                    if (timer2.time() > 1) {
//                        arm.rest();
//                    }
//                    if (arm.atAngle() & timer2.time() > 1) {
//                        previousState = STATES.SCORE_YELLOW;
//                    }
//                } else {
//                    currentState = STATES.PARK;
//                }
//                break;
//            case PARK:
//                if (previousState != currentState) {
//                    arm.wristIn();
//                    park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                            .strafeRight(parkSide*24)
//                            .turn(Math.toRadians(-90))
//                            .build();
//                    drive.followTrajectorySequenceAsync(park);
//                    previousState = STATES.PARK;
//                } else  if (!drive.isBusy()) {
//                    currentState = STATES.STOP;
//                }
//                break;
//            case STOP:
//                break;
//        }
//
//        // see if this works, i kind of doubt some arm functions will work (like the timer states, those seem weird)
//
//
//        arm.update();
//        drive.update();
//        telemetry.addData("power", arm.getPivotPower());
//        telemetry.addData("target", arm.getTargetPivotPos());
//        telemetry.addData("pose", arm.getPivotPos());
//        telemetry.addData("State", currentState);
//        telemetry.addData("is started?", isTimerStarted);
//        telemetry.update();
//    }
//
//    private void setupIMU() {
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT; // TODO: Update once it's done being built
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//    }
//    private void setupDistanceSensor() {
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
//    }
//
//    private void teleLogging(String s) {
//        telemetry.addLine(s);
//        RobotLog.d(s);
//    }
//    private void teleData(String s, String format, Object... args) {
//        telemetry.addData(s, format, args);
//        String stringArguments = String.format(format, args);
//        RobotLog.d(s + ": " + stringArguments);
//    }
//    private void teleData(String s, Object... args) {
//        telemetry.addData(s, args);
//        String stringArguments = String.valueOf(args);
//        RobotLog.d(s + ": " + stringArguments);
//    }
//
//}