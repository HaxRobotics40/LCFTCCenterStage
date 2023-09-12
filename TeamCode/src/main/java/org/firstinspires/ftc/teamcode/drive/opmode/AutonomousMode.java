package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Autonomous
public class AutonomousMode extends LinearOpMode {
    VisionPortal.Builder vPortalBuilder;
    VisionPortal vPortal;

    AprilTagProcessor aprilTagProcessor;
    AprilTagProcessor.Builder aprilTagProcessorBuilder;

    SampleMecanumDrive drive;
    int targetTag;
    double detX; //goal 0, means looking straight at tag
    double detY; // goal ?? depends per robot
    double targetY = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        vPortalBuilder = new VisionPortal.Builder();
        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        aprilTagProcessor = aprilTagProcessorBuilder.build();

        vPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vPortalBuilder.addProcessor(aprilTagProcessor);
        vPortal = vPortalBuilder.build();

        drive = new SampleMecanumDrive(hardwareMap);

        // the team piece has to be entirely red or blue except for letters, and letters cant be greater than 0.5 in
        // shades of red? shiny/dull red? or make distinct red/blue colored outlinen that's very recognizable
        // may need to code own processor, like in FF (see: VisionProcessor -> VisionProccesorInternal, similar structure)
        // tags different for each side

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {


                List<AprilTagDetection> allDets = aprilTagProcessor.getDetections();
                for (AprilTagDetection det : allDets) {

                    // targetTag isolated
                    if (det.id % 3 == targetTag || det.id % 3 == targetTag-3) {
                        detX = det.ftcPose.x;
                        detY = det.ftcPose.y;

                        // assumign using sample mecanum drive, RR
                        //assuming robot made a perfect 90 deg turn and is just to the right or left of tag
                        while(detX !=0) {
                            if (detX > 0) {
                                drive.setMotorPowers(detX/10,-detX/10,detX/10,-detX/10);
                            } else {
                                drive.setMotorPowers(-detX/10,detX/10,-detX/10,detX/10);
                            }
                            this.sleep(20);
                        }


                        while(detY != targetY) {
                            if (detX > targetY) {
                                drive.setMotorPowers(0.1,0.1,0.1,0.1); // do something about this later
                            } else {
                                drive.setMotorPowers(-0.1,-0.1,-0.1,-0.1);
                            }
                            this.sleep(20);

                        }

                        // lol this isn't gonna work
                        break;
                    }
                }




            }
        }



    }


    public void initRobot() {
        
    }
}