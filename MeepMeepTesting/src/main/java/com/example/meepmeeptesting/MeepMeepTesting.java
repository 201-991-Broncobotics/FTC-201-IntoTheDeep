package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {

    public static double tileLength = 23.5625;

    public static Vector2d tileCoords(double X, double Y) { // convert field coords in tiles to coords in inches
        return new Vector2d(X * tileLength, Y * tileLength);
    }

    public static double tiles(double tiles) { // convert number of tiles to inches
        return tiles * tileLength;
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.PI * 1, Math.PI * 1, 6)
                .setDimensions(14.173228, 16.633465)
                .build();


        Pose2d startPose1 = new Pose2d(new Vector2d(tiles(0.5), tiles(-3) + 7.09), Math.toRadians(90));
        Pose2d startPose2 = new Pose2d(new Vector2d(tiles(-0.5), tiles(-3) + 7.09), Math.toRadians(90));
        // Start


        VelConstraint MaxSpeedVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60.0),
                new AngularVelConstraint(Math.PI)
        ));

        VelConstraint SlowDownVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint SlowDownAccelConstraint = new ProfileAccelConstraint(-30.0, 30.0);


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(tileCoords(1.7, 0), Math.toRadians(90)))
                .strafeToConstantHeading(tileCoords(1.7, -0.8))
                .splineToConstantHeading(tileCoords(0.8, -1.7), Math.toRadians(180))
                .strafeToConstantHeading(tileCoords(-1.1, -1.7))
                .splineToLinearHeading(new Pose2d(tileCoords(-1.1, 0), Math.toRadians(0)), Math.toRadians(0))


                .build());


        /*

        myBot.runAction(myBot.getDrive().actionBuilder(startPose1)
                .strafeToConstantHeading(tileCoords(0.25, -1.3))

                .waitSeconds(0.75)

                .setTangent(Math.toRadians(-45)) // bezier curve
                .splineToConstantHeading(tileCoords(2.05, -0.5), Math.toRadians(0))

                .strafeToConstantHeading(tileCoords(2.05, -2.3))
                .strafeToConstantHeading(tileCoords(2.05, -0.7))
                .splineToConstantHeading(tileCoords(2.5, -0.5), Math.toRadians(0))
                .strafeToConstantHeading(tileCoords(2.5, -2.3))
                .strafeToConstantHeading(tileCoords(2.5, -0.7))
                .splineToConstantHeading(tileCoords(2.7, -0.5), Math.toRadians(0))
                .strafeToConstantHeading(tileCoords(2.7, -2.4))

                .strafeToConstantHeading(tileCoords(2, -2.4))
                .strafeToConstantHeading(tileCoords(2, -2.7))

                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.18, -1.3), Math.toRadians(90))

                .setTangent(Math.toRadians(360-55))
                .splineToConstantHeading(tileCoords(2, -2.4), Math.toRadians(360-35))
                .strafeToConstantHeading(tileCoords(2, -2.7))

                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(tileCoords(0.1, -1.3), Math.toRadians(90))

                .setTangent(Math.toRadians(360-60))
                .splineToConstantHeading(tileCoords(2.1, -2.5), Math.toRadians(360-20))

                .build());



         */





        /*


        // Basket Side
        myBot.runAction(myBot.getDrive().actionBuilder(startPose2)
                .strafeToConstantHeading(tileCoords(-0.2, -1.5))
                .strafeToConstantHeading(tileCoords(-0.2, -1.3))

                .waitSeconds(0.75)

                .setTangent(Math.toRadians(180+45))
                .splineToLinearHeading(new Pose2d(tileCoords(-1.5, -1.6), Math.toRadians(135)), Math.toRadians(180))

                .waitSeconds(0.75)

                .strafeToLinearHeading(tileCoords(-2.5, -2.5), Math.toRadians(225))

                .waitSeconds(1.5)

                .strafeToLinearHeading(tileCoords(-1.95, -1.6), Math.toRadians(135))

                .waitSeconds(0.75)

                .strafeToLinearHeading(tileCoords(-2.5, -2.5), Math.toRadians(225))

                .waitSeconds(1.5)

                .strafeToLinearHeading(tileCoords(-2.4, -1.6), Math.toRadians(135))

                .waitSeconds(0.75)

                .strafeToLinearHeading(tileCoords(-2.5, -2.5), Math.toRadians(225))

                .waitSeconds(1.5)

                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(tileCoords(-1, -0.5), Math.toRadians(0)), Math.toRadians(0))

                .build());



         */





        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}