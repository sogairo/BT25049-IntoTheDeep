package com.example.meep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61.5,  Math.toRadians(90)))
                /*
                // preload
                .lineToY(-34)

                // away
                .strafeToLinearHeading(new Vector2d(20, -40), Math.toRadians(-90))

                // sample 1 push
                .splineToConstantHeading(new Vector2d(39, -9), Math.PI / 3)
                .splineToConstantHeading(new Vector2d(48, -45), Math.PI / 2)

                // sample 2 push
                .splineToConstantHeading(new Vector2d(50, -9), Math.PI / 3)
                .splineToConstantHeading(new Vector2d(58, -45), Math.PI / 2)

                // sample 3 push
                .splineToConstantHeading(new Vector2d(57, -9), Math.PI / 3)
                .splineToConstantHeading(new Vector2d(59, -45), Math.PI / 2)

                // we load
                .splineToLinearHeading(new Pose2d(23, -61.5, Math.toRadians(0)), -Math.PI / 2)

                // specimen 1
                .strafeToLinearHeading(new Vector2d(1, -33), Math.toRadians(90))

                // back
                .strafeToLinearHeading(new Vector2d(23, -61.5), Math.toRadians(0))

                // specimen 2
                .strafeToLinearHeading(new Vector2d(2, -33), Math.toRadians(90))

                // park
                .strafeToConstantHeading(new Vector2d(56, -61.5))

                 */

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}