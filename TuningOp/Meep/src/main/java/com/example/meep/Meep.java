package com.example.meep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Meep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // this is the configuration part / tuning
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .build();

        // actual path
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 61.5, Math.toRadians(-90)))
                // put on specimen
                .lineToY(33)

                                .strafeTo(new Vector2d(-48.1, 39))
                .strafeTo(new Vector2d(-58.5, 39))

                /*
                // push first sample
                .setTangent(Math.toRadians(180))
                .lineToX(-10)

                .splineTo(new Vector2d(-48, 14), Math.toRadians(-180))

                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(14)

                // push second sample
                .setTangent(Math.toRadians(180))
                .lineToX(-58)

                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(14)

                // push third sample
                .setTangent(Math.toRadians(180))
                .lineToX(-61)

                .setTangent(Math.toRadians(90))
                .lineToY(50)
                 */

                // get into dropping position
                .setTangent(Math.toRadians(180))
                .strafeTo(new Vector2d(-25, 61))
                .turn(Math.toRadians(-90))

                // the hang 1
                .strafeToLinearHeading(new Vector2d(-3, 35), Math.toRadians(270))

                // back
                .strafeToLinearHeading(new Vector2d(-25, 61), Math.toRadians(180))

                // hang 2
                .strafeToLinearHeading(new Vector2d(-2, 35), Math.toRadians(270))

                // back
                .strafeToLinearHeading(new Vector2d(-25, 61), Math.toRadians(180))

                // hang 3
                .strafeToLinearHeading(new Vector2d(-1, 35), Math.toRadians(270))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}