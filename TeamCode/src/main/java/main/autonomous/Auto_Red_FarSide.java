
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

        import android.util.Size;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
        import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
        import org.firstinspires.ftc.vision.VisionPortal;
        import org.firstinspires.ftc.vision.tfod.TfodProcessor;

        import java.util.List;

        import main.OpModes.Version1_OpMode;


@Autonomous(name="Auto_Red_FarSide")
public class Auto_Red_FarSide extends LinearOpMode {

    // Initialize Variables
    private Servo AUTOHOLDER, CLAMP1, CLAMP2, HOLDER_ROTATE;
    private DcMotor MOTOR_LEFT_LINEARRACK, MOTOR_RIGHT_LINEARRACK;
    int linearRackHighPos = 2900;
    int linearRackHomePos = 0;

    double holderHomePos = 0.14;
    double holderFlippedPos = 0.5;
    double holderPos = holderHomePos;
    double clamp1ClosePos = 0.8;
    double clamp2ClosePos = 0.9;
    double clampOpenPos = 0.0;
    double clamp1Pos = clamp1ClosePos;
    double clamp2Pos = clampOpenPos;
    double autoHolderHoldPos = 0.7;
    double autoHolderReleasePos = 1;
    double x,y;
    int position;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "FTCRobotController/assets/model.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blueTeamProp",
            "redTeamProp",
            "team prop"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {

        // Hardware map all necessary motors and servos
        AUTOHOLDER = hardwareMap.get(Servo.class, "AUTOHOLDER");
        MOTOR_LEFT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-LEFT-LINEARRACK");
        MOTOR_RIGHT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-RIGHT-LINEARRACK");
        CLAMP1 = hardwareMap.get(Servo.class, "CLAMP1");
        CLAMP2 = hardwareMap.get(Servo.class, "CLAMP2");
        HOLDER_ROTATE = hardwareMap.get(Servo.class, "HOLDER-ROTATE");

        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set servos to initialized positions
        HOLDER_ROTATE.setPosition(holderPos - 0.06);   //main arm flip
        CLAMP1.setPosition(clamp1Pos);          //front pixel clamp
        CLAMP2.setPosition(clamp2Pos);          //back pixel clamp
        AUTOHOLDER.setPosition(autoHolderHoldPos);

        //Create road runner trajectories
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -62, Math.toRadians(90));                       // Starting Position 12,-62 heading 90 degrees
        drive.setPoseEstimate(startPose);

        // Team prop is on the left
        TrajectorySequence position1_p1 = drive.trajectorySequenceBuilder(startPose)            // Create trajectory for left prop position.forward(30)
                .forward(30)
                .turn(Math.toRadians(90))                                                       // Move to prop and deposit pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(2)
                .back(2)
                .strafeRight(20)
                .splineTo(new Vector2d(35, -10), Math.toRadians(-90))                     // Move to side of backdrop (avoiding other team)
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(45, -20, Math.toRadians(0)))              // Move in front of backdrop
                .build();

        TrajectorySequence position1_p2 = drive.trajectorySequenceBuilder(position1_p1.end())
                .waitSeconds(8)
                .strafeLeft(20)                                                        // Drive into park zone
                .forward(10)
                .build();

        // Team prop is in the middle
        TrajectorySequence position2_p1 = drive.trajectorySequenceBuilder(startPose)               // Create trajectory for middle prop position
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Move forward and drop of pixel
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(2)
                .back(4)
                .strafeLeft(20)                                                        // Avoid other team
                .forward(20)
                .splineTo(new Vector2d(43, -15), Math.toRadians(0))                      // Move to backdrop and wait at the side (for other team)
                .waitSeconds(8)
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)))              // Move to the front of the backdrop
                .build();

        TrajectorySequence position2_p2 = drive.trajectorySequenceBuilder(position2_p1.end())
                //Drive into the parking zone
                .strafeLeft(23)
                .forward(10)
                .build();

        // Team prop on the right
        TrajectorySequence position3_p1 = drive.trajectorySequenceBuilder(startPose)            // Create trajectory for right prop position
                .forward(30)                                                         // Drive to prop and deposit pixel
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(2)
                .back(4)
                .strafeLeft(20)
                .splineToConstantHeading(new Vector2d(35, -10), Math.toRadians(90))    // Drive next to backdrop and wait (for other team)
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(53, -44, Math.toRadians(0)))           // Drive in front of backdrop
                .build();

        TrajectorySequence position3_p2 = drive.trajectorySequenceBuilder(position3_p1.end())
                .waitSeconds(8)
                .strafeLeft(28)                                                   // Drive into parking zone
                .forward(10)
                .build();

        //Scan for prop
        initTfod();

        if (!isStarted() && !isStopRequested() && !opModeIsActive()) {                    //does this run?
            while (!isStarted() && !isStopRequested() && !opModeIsActive()){

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
        }
        visionPortal.close();

        waitForStart();

        if (!isStopRequested()) {
            // When program starts, run appropriate trajectory
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            if (position == 1) {
                drive.followTrajectorySequence(position1_p1);
                deliverPixel();
                drive.followTrajectorySequence(position1_p2);
            } else if (position == 3) {
                drive.followTrajectorySequence(position3_p1);
                deliverPixel();
                drive.followTrajectorySequence(position3_p2);
            } else {
                drive.followTrajectorySequence(position2_p1);
                deliverPixel();
                drive.followTrajectorySequence(position2_p2);
            }
        }
    }
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam")); //check
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("- Position", "%.0f / %.0f", x, y);

            if (x > 10 && x < 100) {
                position = 1;
            } else if (x > 200 && x < 300) {
                position = 3;
            } else {
                position = 2;
            }
        }
    }

    public void deliverPixel() {
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MOTOR_LEFT_LINEARRACK.setPower(-1);
        MOTOR_RIGHT_LINEARRACK.setPower(1);
        sleep(2000);

        int position1 = MOTOR_LEFT_LINEARRACK.getCurrentPosition();

        if (position1 == linearRackHomePos) {//margin of error
            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);

            MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            MOTOR_LEFT_LINEARRACK.setPower(-1);
            MOTOR_RIGHT_LINEARRACK.setPower(1);

            sleep(2000);
            position1 = MOTOR_LEFT_LINEARRACK.getCurrentPosition();
        }


        HOLDER_ROTATE.setPosition(holderFlippedPos);
        sleep(500);
        CLAMP1.setPosition(clampOpenPos);
        sleep(500);
        HOLDER_ROTATE.setPosition(holderHomePos);
        sleep(500);

        MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHomePos);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHomePos);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MOTOR_LEFT_LINEARRACK.setPower(-1);
        MOTOR_RIGHT_LINEARRACK.setPower(1);

        sleep(2000);
    }
}





