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
                                .forward(30)                                                           // Drive to prop and deposit pixel
                                .turn(Math.toRadians(-90))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    //AUTOHOLDER.setPosition(autoHolderReleasePos);
                                })
                                .waitSeconds(2)
                                .back(4)
                                .strafeLeft(20)
                                .lineToLinearHeading(new Pose2d(40, 20, Math.toRadians(0)))       // Drive next to backdrop and wait (for other team)
                                .waitSeconds(8)
                                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(0)))             // Drive in front of backdrop
                                .turn(10) //check
                                .forward(5)           // Move to the front of the backdrop
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}