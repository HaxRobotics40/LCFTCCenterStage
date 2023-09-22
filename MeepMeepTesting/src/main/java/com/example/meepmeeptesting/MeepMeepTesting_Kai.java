package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting_Kai {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 60, Math.toRadians(720), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .build()
                );
        Image img = null;
        try { img = ImageIO.read(new File("/Users/kaimasujima/Downloads/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}