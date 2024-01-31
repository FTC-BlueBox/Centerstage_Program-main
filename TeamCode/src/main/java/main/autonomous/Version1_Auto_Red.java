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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import main.OpModes.Version1_OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name="auto 1", group="Robot")
public class Version1_Auto_Red extends OpMode {
    private DcMotor MOTOR1, MOTOR2, MOTOR3, MOTOR4;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double MotorPower = 0.4;

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    OpenCvWebcam webcam = null;

    @Override
    public void init(){

        //creating webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(new examplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened(){
                webcam.startStreaming(1024,576, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){
            }
        });
    }

    
    @Override //will any of this work?!
    public void loop(){
        MOTOR1  = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3  = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");

        telemetry.update();

        // Step 1:  Drive forward for 3 seconds
        /*MOTOR1.setPower(MotorPower);
        MOTOR2.setPower(MotorPower);
        MOTOR3.setPower(-MotorPower);
        MOTOR4.setPower(-MotorPower);
        sleep(2000);
        MOTOR1.setPower(0);
        MOTOR2.setPower(0);
        MOTOR3.setPower(0);
        MOTOR4.setPower(0);*/
    }
    public static class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColorRed = new Scalar(255.0,0.0,0.0);
        Scalar rectColorBlue = new Scalar(0.0,0.0,255.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
           // telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,340,575);
            Rect middleRect = new Rect(340,1,679,575);
            Rect rightRect = new Rect(679,1,1023,575);

            input.copyTo(outPut);
            if(Version1_OpMode.ALLIANCE_COLOR == "blue"){
                Imgproc.rectangle(outPut,leftRect,rectColorBlue,2);
                Imgproc.rectangle(outPut, rightRect, rectColorBlue, 2);
                Imgproc.rectangle(outPut, middleRect, rectColorBlue, 2);
            } else if (Version1_OpMode.ALLIANCE_COLOR == "red"){
                Imgproc.rectangle(outPut,leftRect,rectColorRed,2);
                Imgproc.rectangle(outPut, rightRect, rectColorRed, 2);
                Imgproc.rectangle(outPut, middleRect, rectColorBlue, 2);
            }
            
            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect)

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(middleCrop, middleCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Slalar middleavg = Core.mean(middleCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            middleavgfin = middleavg.val[0];


            if(leftavgfin < rightavgfin && leftavgfin < middleavgfin){
                telemetry.addLine("1");
                Version1_OpMode.propPosition = 1;
            }else if (leftavgfin > rightavgfin && leftavgfin > middleavgfin){
                telemetry.addLine("3");
                Version1_OpMode.propPosition = 3;
            }else{
                Version1_OpMode.propPosition = 2;
                telemetry.addLine("2");
            }

            return(outPut);
        }

    }

}
