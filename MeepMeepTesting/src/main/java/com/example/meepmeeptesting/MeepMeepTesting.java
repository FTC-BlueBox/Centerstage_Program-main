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
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(180)))

                                .back(24)
                                .turn(Math.toRadians(90))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                                    // MOTOR_INTAKE.setPower(-1);
                                   // GROUND_WHEELS.setPower(-1);
                                })
                                .waitSeconds(1)
                                .forward(5)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                                    //MOTOR_INTAKE.setPower(0);
                                    //GROUND_WHEELS.setPower(0);
                                })
                                .waitSeconds(1)
                                .forward(37)
                                .waitSeconds(1)
                                .forward(2)
                                .build()                     // Drive to backdrop

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}