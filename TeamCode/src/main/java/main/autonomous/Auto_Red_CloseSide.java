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

@Autonomous(name="Auto_Red_CloseSide", group="Robot")
public class Auto_Red_CloseSide extends LinearOpMode {

    private Servo AUTOHOLDER, CLAMP1,CLAMP2, HOLDER_ROTATE;
    private DcMotor MOTOR_LEFT_LINEARRACK, MOTOR_RIGHT_LINEARRACK;
    int linearRackHighPos = 2900;
    int linearRackHomePos = 0;

    OpenCvWebcam webcam = null;

    double leftavgfin;
    double rightavgfin;
    double middleavgfin;

    public class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;

        Mat outPut = new Mat();
        Scalar rectColorRed = new Scalar(255.0,0.0,0.0);
        Scalar rectColorBlue = new Scalar(0.0,0.0,255.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            // telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,210,479);
            Rect middleRect = new Rect(210,1,210,479);
            Rect rightRect = new Rect(420,1,210,479);
            //Rect leftRect = new Rect(1,1,510,575);
            //Rect rightRect = new Rect(510,1,1023,575);


            input.copyTo(outPut);
            Version1_OpMode.ALLIANCE_COLOR = "red"; //fix this, then add ifs
            Imgproc.rectangle(outPut,leftRect,rectColorRed,2);
            Imgproc.rectangle(outPut, rightRect, rectColorRed, 2);
            Imgproc.rectangle(outPut, middleRect, rectColorRed, 2);


            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(middleCrop, middleCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar middleavg = Core.mean(middleCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            middleavgfin = middleavg.val[0];

            telemetry.addData("right",rightavgfin);
            telemetry.addData("left",leftavgfin);
            telemetry.addData("middle",middleavgfin);



            return(outPut);
        }

    }

    @Override
    public void runOpMode() {
        //creating webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(new examplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened(){
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                if(leftavgfin > rightavgfin && leftavgfin > middleavgfin){
                    telemetry.addLine("1");
                    Version1_OpMode.propPosition = 1;
                }else if (leftavgfin < rightavgfin && rightavgfin > middleavgfin){
                    telemetry.addLine("3");
                    Version1_OpMode.propPosition = 3;
                }else{
                    Version1_OpMode.propPosition = 2;
                    telemetry.addLine("2");
                }
            }
            public void onError(int errorCode){
            }
        });


        double holderHomePos = 0.14;
        double holderFlippedPos = 0.5;
        double holderPos = holderHomePos;
        double clamp1ClosePos = 1;
        double clamp2ClosePos = 0.9;
        double clampOpenPos = 0.0;
        double clamp1Pos = clamp1ClosePos;
        double clamp2Pos = clampOpenPos;
        double autoHolderHoldPos = 0.0;
        double autoHolderReleasePos = 0.4;

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


        HOLDER_ROTATE.setPosition(holderPos);   //main arm flip
        CLAMP1.setPosition(clamp1Pos);          //front pixel clamp
        CLAMP2.setPosition(clamp2Pos);          //back pixel clamp
        AUTOHOLDER.setPosition(autoHolderHoldPos - 0.03);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90)); //starts bot at x - 10 y - -8 heading 90 degrees
        drive.setPoseEstimate(startPose);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Ready to run");    //
        telemetry.addData("Position: ", Version1_OpMode.propPosition);
        telemetry.update();


        //!!APRIL TAGS!!!!

        //when team prop is in the middle
        TrajectorySequence position2 = drive.trajectorySequenceBuilder(startPose)
                /*.forward(30)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(1)
                .back(6)
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))*/
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);

                    MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    if (MOTOR_RIGHT_LINEARRACK.isBusy())  MOTOR_RIGHT_LINEARRACK.setPower(0.6);
                    else                                  MOTOR_RIGHT_LINEARRACK.setPower(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);

                    MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (MOTOR_LEFT_LINEARRACK.isBusy())  MOTOR_LEFT_LINEARRACK.setPower(-0.6);
                    else                                 MOTOR_LEFT_LINEARRACK.setPower(0);

                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    HOLDER_ROTATE.setPosition(holderFlippedPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                    CLAMP1.setPosition(clampOpenPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    HOLDER_ROTATE.setPosition(holderHomePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                    bringHomeLinearRack();
                })
                .waitSeconds(8)

                /*
                .strafeRight(23)
                .forward(10)*/
                .build();

        //when team prop is on the right
        TrajectorySequence position3 = drive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(5)
                .forward(80)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    liftLinearRack();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    HOLDER_ROTATE.setPosition(holderFlippedPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    CLAMP1.setPosition(clampOpenPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                    HOLDER_ROTATE.setPosition(holderHomePos - 0.03);
                })
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    bringHomeLinearRack();
                })
                .waitSeconds(6)
                .strafeLeft(23)
                .forward(10)
                .build();

        //when team prop is on the left
        TrajectorySequence position1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(3)
                .forward(28)
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    AUTOHOLDER.setPosition(autoHolderReleasePos);
                })
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(40, -34, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    liftLinearRack();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    HOLDER_ROTATE.setPosition(holderFlippedPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    CLAMP1.setPosition(clampOpenPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                    HOLDER_ROTATE.setPosition(holderHomePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(4, () -> {
                    bringHomeLinearRack();
                })
                .waitSeconds(6)
                .strafeLeft(23)
                .forward(10)
                .build();

        waitForStart();
        if (!isStopRequested())

            if (Version1_OpMode.propPosition == 1) {
                drive.followTrajectorySequence(position2);
            }
            else if (Version1_OpMode.propPosition == 3) {
                drive.followTrajectorySequence(position3);
            }else {
                drive.followTrajectorySequence(position2);
            }

    }

    public void liftLinearRack(){
        MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (MOTOR_LEFT_LINEARRACK.isBusy())  MOTOR_LEFT_LINEARRACK.setPower(-1);
        else                                 MOTOR_LEFT_LINEARRACK.setPower(0);

        if (MOTOR_RIGHT_LINEARRACK.isBusy())  MOTOR_RIGHT_LINEARRACK.setPower(1);
        else                                  MOTOR_RIGHT_LINEARRACK.setPower(0);

    }
    public void bringHomeLinearRack(){
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

    }

}