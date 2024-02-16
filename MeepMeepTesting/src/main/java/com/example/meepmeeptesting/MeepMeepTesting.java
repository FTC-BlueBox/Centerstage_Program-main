package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -62, Math.toRadians(90)))
                                .forward(30)
                                .turn(Math.toRadians(90))                                                       // Move to prop and deposit pixel
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                   // AUTOHOLDER.setPosition(autoHolderReleasePos);
                                })
                                .waitSeconds(2)
                                .back(2)
                                .strafeRight(30)
                                .turn(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(35, -10), Math.toRadians(0))                     // Move to side of backdrop (avoiding other team)
                                .waitSeconds(8)
                                .lineToLinearHeading(new Pose2d(45, -20, Math.toRadians(0)))              // Move in front of backdrop
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}