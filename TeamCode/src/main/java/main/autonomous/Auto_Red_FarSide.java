
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
        import com.qualcomm.robotcore.hardware.CRServo;
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


//@Autonomous(name="Auto_Red_FarSide")
public class Auto_Red_FarSide extends LinearOpMode {

    // Initialize Variables
    private Servo CLAMP1, CLAMP2, BOX_FLIP, INTAKE_LIFT;
    private CRServo GROUND_WHEELS;
    private DcMotor MOTOR_LEFT_LINEARRACK, MOTOR_RIGHT_LINEARRACK, MOTOR_INTAKE;
    int linearRackHighPos = 1500;
    int linearRackHomePos = 0;
    double boxHomePos = 1;                             // Top intake rotator positions
    double boxFlippedPos = 0.4;
    double boxPos = boxHomePos;

    double clamp1ClosePos = 0.35;                                // Pixel clamp positions
    double clamp1OpenPos = 0.0;
    double clamp2OpenPos = 0.1;
    double clamp2ClosePos = 0.4;
    double clamp1Pos = clamp1OpenPos;
    double clamp2Pos = clamp2OpenPos;
    double x,y;
    int position;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model.tflite";

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
        MOTOR_LEFT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-LEFT-LINEARRACK");
        MOTOR_RIGHT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-RIGHT-LINEARRACK");
        CLAMP1 = hardwareMap.get(Servo.class, "CLAMP1");
        CLAMP2 = hardwareMap.get(Servo.class, "CLAMP2");
        BOX_FLIP = hardwareMap.get(Servo.class, "BOX-FLIP");
        INTAKE_LIFT = hardwareMap.get(Servo.class, "INTAKE-LIFT");
        GROUND_WHEELS = hardwareMap.get(CRServo.class, "GROUND-WHEELS");
        MOTOR_INTAKE = hardwareMap.get(DcMotor.class, "MOTOR-INTAKE");

        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set servos to initialized positions
        CLAMP1.setPosition(clamp1ClosePos);                 // front pixel clamp
        CLAMP2.setPosition(clamp2ClosePos);                 // back pixel clamp
        BOX_FLIP.setPosition(boxPos);
        INTAKE_LIFT.setPosition(1);


        //Create road runner trajectories
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -62, Math.toRadians(90));                      // Starting Position 12,-62 heading 90 degrees
        drive.setPoseEstimate(startPose);

        // Team prop is on the left
        TrajectorySequence position1_p1 = drive.trajectorySequenceBuilder(startPose)            // Create trajectory for left prop position
                .back(24)
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                    MOTOR_INTAKE.setPower(-1);
                    GROUND_WHEELS.setPower(-1);
                })
                .waitSeconds(1)
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                    MOTOR_INTAKE.setPower(0);
                    GROUND_WHEELS.setPower(0);
                })
                .waitSeconds(1)
                .build();

        // Team prop is in the middle
        TrajectorySequence position2_p1 = drive.trajectorySequenceBuilder(startPose)             // Create trajectory for middle prop position
                .back(23)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                    MOTOR_INTAKE.setPower(-1);
                    GROUND_WHEELS.setPower(-1);
                })
                .waitSeconds(1)
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                    MOTOR_INTAKE.setPower(0);
                    GROUND_WHEELS.setPower(0);
                })

                .build();

        // Team prop on the right
        TrajectorySequence position3_p1 = drive.trajectorySequenceBuilder(startPose)             // Create trajectory for right prop position
                .back(24)
                .turn(Math.toRadians(-90))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                    MOTOR_INTAKE.setPower(-1);
                    GROUND_WHEELS.setPower(-1);
                })
                .waitSeconds(1)
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {                            // Run to prop and release pixel
                    MOTOR_INTAKE.setPower(0);
                    GROUND_WHEELS.setPower(0);
                })
                .waitSeconds(1)
                .build();

        //Scan for prop
        initTfod();

        if (!isStarted() && !isStopRequested() && !opModeIsActive()) {
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
            INTAKE_LIFT.setPosition(0.05);
            sleep(1000);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            if (position == 1) {
                drive.followTrajectorySequence(position1_p1);
            } else if (position == 3) {
                drive.followTrajectorySequence(position3_p1);
            } else {
                drive.followTrajectorySequence(position2_p1);
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

            if (x > 0 && x < 175) {
                position = 1;
            } else if (x > 475 && x < 600) {
                position = 3;
            } else { //175-475
                position = 2;
            }
            telemetry.addData("Position: ", position);
        }
    }

    public void deliverPixel() {                                                    // Method to lift linear rack, score pixel, bring it down
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MOTOR_LEFT_LINEARRACK.setTargetPosition(linearRackHighPos);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(-linearRackHighPos);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MOTOR_LEFT_LINEARRACK.setPower(-1);
        MOTOR_RIGHT_LINEARRACK.setPower(1);
        sleep(1000);

        int position1 = MOTOR_LEFT_LINEARRACK.getCurrentPosition();

        if (position1 == linearRackHomePos) {
            MOTOR_LEFT_LINEARRACK.setTargetPosition(linearRackHighPos);
            MOTOR_RIGHT_LINEARRACK.setTargetPosition(-linearRackHighPos);

            MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            MOTOR_LEFT_LINEARRACK.setPower(-1);
            MOTOR_RIGHT_LINEARRACK.setPower(1);
            sleep(1000);
            position1 = MOTOR_LEFT_LINEARRACK.getCurrentPosition();
        }

        BOX_FLIP.setPosition(boxFlippedPos);
        sleep(1600);
        CLAMP1.setPosition(clamp1OpenPos);
        sleep(500);
        BOX_FLIP.setPosition(boxHomePos);
        sleep(1000);

        MOTOR_LEFT_LINEARRACK.setTargetPosition(linearRackHomePos);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(-linearRackHomePos);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MOTOR_LEFT_LINEARRACK.setPower(-1);
        MOTOR_RIGHT_LINEARRACK.setPower(1);

        sleep(1000);
    }
}