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

        //Pose2d sampleint = new Pose2d(-38, -61.5, Math.toRadians(-90));

        // actual path
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 61.5, Math.toRadians(-90)))
                                .lineToY(41)
                /*
                // preload
                .strafeToLinearHeading(new Vector2d(-57, -56), Math.toRadians(225))

                // sample 1
                .strafeToLinearHeading(new Vector2d(-46.5, -45), Math.toRadians(90))

                // back
                .strafeToLinearHeading(new Vector2d(-57, -56), Math.toRadians(225))

                //
                .strafeToLinearHeading(new Vector2d(-57, -45), Math.toRadians(90))

                // back
                .strafeToLinearHeading(new Vector2d(-57, -56), Math.toRadians(225))

                 */
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}