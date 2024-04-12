/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package main.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="auto 2", group="Robot")
public class RoadRunner_Practice extends LinearOpMode {
    private DcMotor MOTOR1, MOTOR2, MOTOR3, MOTOR4;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double MotorPower = 0.4;

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10,-8, Math.toRadians(90)); //starts bot at x - 10 y - -8 heading 90 degrees
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), true) //true indicates running in reverse
                .strafeRight(10)
                .build();

        //start has to take into consideration a turn inbetween for a spline
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90)))) //each start must be the previous end
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0)

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0,
                //slows down to max 15 in/s
                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(traj1); //each wont run until the previous is complete
        //robot.dropServo();
        drive.followTrajectory(traj2);
        drive.turn(Math.toRadians(90)); //turning is different than a trajectory
        drive.followTrajectory(traj3);

        //all the different options
        Trajectory traj4 = drive.trajectoryBuilder(traj2.end())
                .forward(40)
                .back(40)
                .strafeLeft(40)
                .strafeRight(40)
                .strafeTo(new Vector2d(40, 40)) //maintains the same heading
                .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90))) //interpolates between start heading and specified end heading
                .splineTo(new Vector2d(40, 40), Math.toRadians(0)) // spline path while following a tangent heading interpolator
                .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0)) // spline while maintaing heading

                //1 temporal markers: runs a specific number of seconds into the program (global)
                .addTemporalMarker(2, () -> {
                })
                    //use an offset to run a temporal marker during a wait (negatives work too)
                    //.UNSTABLE_addTemporalMarkerOffset(3, () -> {}) 3 seconds after the last call
                //2 Displacement Markers: runs after the first splineTo() (order matters)
                .addDisplacementMarker(() -> {
                })
                    //alternate, runs 20 inches into the program (global)
                    .addDisplacementMarker(20, () -> {
                     })
                //3 Spatial Markers: runs at a specifed coordinate
                .addSpatialMarker(new Vector2d(20, 20), () -> {
                })

                .build();




        //Instead of listing out all the trajectories, make a trajectory sequence:
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //example
                .splineTo(new Vector2d(10, 10), 0)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(25, -15), 0)
                .waitSeconds(3)         //pauses?
                .turn(Math.toRadians(45))
                .forward(10)
                .strafeRight(5)
                .turn(Math.toRadians(90))
                .strafeLeft(5)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)), 0)
                .build();

        // .setReversed(boolean) runs entire bot backwards
        //.setConstraints(TrajectoryVelocityConstraint, TrajectoryAccelerationConstraint) constrains max velocity
        //.resetConstraints() resets
        //.setVelConstraint(TrajectoryVelocityConstraint) only velocity
        //.resetVelConstraint() resets


        telemetry.addData("linear rack faces backdrop", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


    }
}