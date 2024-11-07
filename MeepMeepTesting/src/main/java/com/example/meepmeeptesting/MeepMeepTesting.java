package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double tileLength = 23.5625;

    public static Vector2d tileCoords(double X, double Y) { // convert field coords in tiles to coords in inches
        return new Vector2d(X * tileLength, Y * tileLength);
    }

    public static double tiles(double tiles) { // convert number of tiles to inches
        return tiles * tileLength;
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14.173228, 16.633465)
                .build();


        Pose2d startPose = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 7.09), Math.toRadians(90));
        // Start


        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .strafeToConstantHeading(tileCoords(0.2, -1.6))
                .waitSeconds(1.5)
                // next trajectory
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(tileCoords(1.4, -1.6), Math.toRadians(45)), Math.toRadians(0))
                // end
                .waitSeconds(2)
                .turnTo(Math.toRadians(-45))
                .waitSeconds(2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}