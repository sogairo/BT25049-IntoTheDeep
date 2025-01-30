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

                // grab spec 1
                .strafeTo(new Vector2d(-15, 35))
                .lineToX(-48)

                // deposit
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(50, Math.toRadians(90))

                // grab spec 2
                .strafeToLinearHeading(new Vector2d(-58.5, 39), Math.toRadians(-90))

                // deposit
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(50, Math.toRadians(90))

                // go grab specimen
                .setTangent(Math.toRadians(0))
                .lineToX(-40)
                .strafeToLinearHeading(new Vector2d(-32.5, 61.5), Math.toRadians(180))

                // put specimen on bar
                .strafeToLinearHeading(new Vector2d(0, 33), Math.toRadians(-90))

                // go back
                .strafeToLinearHeading(new Vector2d(-32.5, 61.5), Math.toRadians(180))

                // put specimen on bar
                .strafeToLinearHeading(new Vector2d(0, 33), Math.toRadians(-90))

                // ascent
                .strafeToLinearHeading(new Vector2d(-40, 50), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .lineToY(10)
                .setTangent(Math.toRadians(0))
                .lineToX(-24)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}