package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import java.io.*;
import java.awt.*;
import javax.imageio.ImageIO;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        int itemSector = 3;
        Pose2d pose2 = new Pose2d(12,-43, Math.toRadians(90+((0-1)*-39.4)));
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -66, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(12, -43, Math.toRadians(90+((0-1)*-39.4))))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(12, Math.signum(pose2.getY())*36, 0))
                                .forward(36)
                                .strafeLeft(5.25)
                                .waitSeconds(4)
                                .strafeRight(18)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}