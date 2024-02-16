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
                        drive.trajectorySequenceBuilder(new Pose2d(-32, 64, Math.toRadians(-90)))
                                .forward(30)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Move forward and drop of pixel
                                    //AUTOHOLDER.setPosition(autoHolderReleasePos);
                                })
                                .waitSeconds(1)
                                .back(4)
                                .strafeRight(20)                                                        // Avoid other team
                                .forward(20)
                                .splineTo(new Vector2d(40, 20), Math.toRadians(0))                      // Move to backdrop and wait at the side (for other team)
                                .waitSeconds(6)
                                .lineToLinearHeading(new Pose2d(47, 40, Math.toRadians(0)))              // Move to the front of the backdrop
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}