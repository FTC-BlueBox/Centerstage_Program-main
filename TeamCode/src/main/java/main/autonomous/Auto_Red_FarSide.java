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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.OpenCV;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import main.OpModes.Version1_OpMode;

@Autonomous(name="Auto_Red_FarSide", group="Robot")
public class Auto_Red_FarSide extends LinearOpMode {

    // Initialize Variables
    private Servo AUTOHOLDER, CLAMP1, CLAMP2, HOLDER_ROTATE;
    private DcMotor MOTOR_LEFT_LINEARRACK, MOTOR_RIGHT_LINEARRACK;
    int linearRackHighPos = 2900;
    int linearRackHomePos = 0;
    double leftavgfin;
    double rightavgfin;
    double middleavgfin;
    OpenCvWebcam webcam = null;
    double holderHomePos = 0.14;
    double holderFlippedPos = 0.5;
    double holderPos = holderHomePos;
    double clamp1ClosePos = 0.8;
    double clamp2ClosePos = 0.9;
    double clampOpenPos = 0.0;
    double clamp1Pos = clamp1ClosePos;
    double clamp2Pos = clampOpenPos;
    double autoHolderHoldPos = 1;
    double autoHolderReleasePos = 0.7;

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

        //Set servos to initialized positions
        HOLDER_ROTATE.setPosition(holderPos);   //main arm flip
        CLAMP1.setPosition(clamp1Pos);          //front pixel clamp
        CLAMP2.setPosition(clamp2Pos);          //back pixel clamp
        AUTOHOLDER.setPosition(autoHolderHoldPos - 0.03);

        //Create webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        examplePipeline pipeline = new examplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {                                                           //open camera
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -62, Math.toRadians(90));                       // Starting Position 12,-62 heading 90 degrees
        drive.setPoseEstimate(startPose);

        // Team prop is on the left
        TrajectorySequence position1_p1 = drive.trajectorySequenceBuilder(startPose)               // Create trajectory for left prop position.forward(30)
                .forward(30)
                .turn(Math.toRadians(90))                                                       // Move to prop and deposit pixel
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(2)
                .back(4)
                .strafeRight(20)
                .splineTo(new Vector2d(35, -10), Math.toRadians(-90))                    // Move to side of backdrop (avoiding other team)
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(48, -15, Math.toRadians(0)))              // Move in front of backdrop
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
                .splineTo(new Vector2d(35, -10), Math.toRadians(0))                      // Move to backdrop and wait at the side (for other team)
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(50, -36, Math.toRadians(0)))              // Move to the front of the backdrop
                .build();

        TrajectorySequence position2_p2 = drive.trajectorySequenceBuilder(position2_p1.end())
                .waitSeconds(8)                                                             //Drive into the parking zone
                .strafeLeft(23)
                .forward(10)
                .build();

        // Team prop on the right
        TrajectorySequence position3_p1 = drive.trajectorySequenceBuilder(startPose)            // Create trajectory for right prop position
                .forward(30)                                                         // Drive to prop and deposit pixel
                .turn(Math.toRadians(-90))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(2)
                .back(4)
                .strafeLeft(20)
                .splineToConstantHeading(new Vector2d(35, -10), Math.toRadians(90))    // Drive next to backdrop and wait (for other team)
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(50, -40, Math.toRadians(0)))           // Drive in front of backdrop
                .build();

        TrajectorySequence position3_p2 = drive.trajectorySequenceBuilder(position3_p1.end())
                .waitSeconds(8)
                .strafeLeft(28)                                                   // Drive into parking zone
                .forward(10)
                .build();


        while (!isStopRequested() && !isStarted()) {
            pipeline.returnPosition();                                                     // Continuously update the prop position during init()
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Ready to run");                       // Update telemetry information
        telemetry.addData("Prop Position: ", Version1_OpMode.getPropPosition());
        telemetry.update();
        sleep(500);

        waitForStart();

        if (!isStopRequested())                                                            // When program starts, run appropriate trajectory
            if (Version1_OpMode.getPropPosition() == 1) {
                drive.followTrajectorySequence(position1_p1);
                deliverPixel();
                drive.followTrajectorySequence(position1_p2);
            }
            else if (Version1_OpMode.getPropPosition() == 3) {
                drive.followTrajectorySequence(position3_p1);
                deliverPixel();
                drive.followTrajectorySequence(position3_p2);
            }else {
                drive.followTrajectorySequence(position2_p1);
                deliverPixel();
                drive.followTrajectorySequence(position2_p2);
            }
    }

    public void deliverPixel(){

        MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (MOTOR_LEFT_LINEARRACK.isBusy())  MOTOR_LEFT_LINEARRACK.setPower(-1);
        else                                 MOTOR_LEFT_LINEARRACK.setPower(0);

        if (MOTOR_RIGHT_LINEARRACK.isBusy())  MOTOR_RIGHT_LINEARRACK.setPower(1);//                                                                              `   `);
        else                                  MOTOR_RIGHT_LINEARRACK.setPower(0);

        sleep(2000);
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

        if (MOTOR_LEFT_LINEARRACK.isBusy())  MOTOR_LEFT_LINEARRACK.setPower(-1);
        else                                 MOTOR_LEFT_LINEARRACK.setPower(0);

        if (MOTOR_RIGHT_LINEARRACK.isBusy())  MOTOR_RIGHT_LINEARRACK.setPower(1);
        else                                  MOTOR_RIGHT_LINEARRACK.setPower(0);

        sleep(2000);

    }
    public class examplePipeline extends OpenCvPipeline {                                 // Create pipeline for opencv camera
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;

        Mat outPut = new Mat();
        Scalar rectColorRed = new Scalar(255.0,0.0,0.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //Change slightly?
            Rect leftRect = new Rect(1,1,210,479);
            Rect middleRect = new Rect(210,1,210,479);
            Rect rightRect = new Rect(420,1,210,479);


            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColorRed,2);                    // Check for red values
            Imgproc.rectangle(outPut, rightRect, rectColorRed, 2);
            Imgproc.rectangle(outPut, middleRect, rectColorRed, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop, 2);                               // Split up screen
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(middleCrop, middleCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar middleavg = Core.mean(middleCrop);

            leftavgfin = leftavg.val[0];                                                  // Average color values
            rightavgfin = rightavg.val[0];
            middleavgfin = middleavg.val[0];

            /*telemetry.addData("right",rightavgfin);
            telemetry.addData("left",leftavgfin);
            telemetry.addData("middle",middleavgfin);*/

            return(outPut);
        }
        public int returnPosition(){                                                     // Set prop position according to averages
            if (leftavgfin > rightavgfin && leftavgfin > middleavgfin) {
                Version1_OpMode.setPropPosition(1);
            } else if (leftavgfin < rightavgfin && rightavgfin > middleavgfin) {
                Version1_OpMode.setPropPosition(3);
            } else {
                Version1_OpMode.setPropPosition(2);
            } return Version1_OpMode.getPropPosition();
        }

    }

}