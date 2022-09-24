package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(9.84252, 9.84252)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 50, Math.toRadians(60), Math.toRadians(60), 18)
                .followTrajectorySequence(drive ->
                        //new Pose2d(36, -66, Math.toRadians(90))
                        //new Pose2d(12, -24, Math.toRadians(180))
                        drive.trajectorySequenceBuilder(new Pose2d(36, -66, Math.toRadians(90)))
                                //TODO: Test new stack
                               // .setReversed(true)
                                //.splineToConstantHeading(new Vector2d(18, -12), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                                //END STACK
//                                .splineToConstantHeading(new Vector2d(30, -60), Math.toRadians(0))
//                               // .strafeTo(new Vector2d(12, -55))
//                                .lineToSplineHeading(new Pose2d(12, -24, Math.toRadians(180)))
                              //  .build();
                                .splineToConstantHeading(new Vector2d(12, -55), Math.toRadians(0))
                               // .strafeTo(new Vector2d(12, -55))
                                //.splineToLinearHeading(new Pose2d(12, -24, Math.toRadians(180)), Math.toRadians(-40))
                                //.splineToConstantHeading(new Vector2d(55, -36), Math.toRadians(-60))
                                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(180)))
                                //.splineToSplineHeading(new Pose2d(58, -24, Math.toRadians(30)), Math.toRadians(34))
                                 //.lineToConstantHeading(new Vector2d(58, -16))
                                //.splineToSplineHeading(new Pose2d(60, -18, Math.toRadians(30)), Math.toRadians(220))

                                //.lineTo(new Vector2d(36, -47))
                               // .lineToConstantHeading(new Vector2d(12, -24))
                               // .splineToSplineHeading(new Pose2d(12, -24, Math.toRadians(180)), Math.toRadians(140))
                               // .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(12, -24), Math.toRadians(-140))
                               // .splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(0))

                                //.splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(-20))
//                                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(-120))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}