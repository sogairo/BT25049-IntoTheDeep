package com.example.visualroadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class VisualRoadRunner {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // this is the configuration part / tuning
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // actual path
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(23, -60, Math.toRadians(90)))
                // put on specimen
                .strafeTo(new Vector2d(0, -36))

                // push first sample
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(25, -36))
                .splineTo(new Vector2d(47.5, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(47.5, -55))

                // push second sample
                .strafeTo(new Vector2d(47.5, -10))
                .strafeTo(new Vector2d(58, -10))
                .strafeTo(new Vector2d(58, -55))

                // push third
                .strafeTo(new Vector2d(58, -10))
                .strafeTo(new Vector2d(61, -10))
                .strafeTo(new Vector2d(61, -55))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}