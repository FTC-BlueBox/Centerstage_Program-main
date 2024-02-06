package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


@Autonomous(name="open cv test", group="")
public class OpenCV extends OpMode{
    
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
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){
            }
        });
    }

    @Override
    public void loop(){

    }

    public class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;
        double leftavgfin;
        double rightavgfin;
        double middleavgfin;
        Mat outPut = new Mat();
        Scalar rectColorRed = new Scalar(255.0,0.0,0.0);
        Scalar rectColorBlue = new Scalar(0.0,0.0,255.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
           // telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,210,480);
            Rect middleRect = new Rect(210,1,210,480);
            Rect rightRect = new Rect(420,1,210,480);
            //Rect leftRect = new Rect(1,1,510,575);
            //Rect rightRect = new Rect(510,1,1023,575);


            input.copyTo(outPut);
            Version1_OpMode.ALLIANCE_COLOR = "red";
            if(Version1_OpMode.ALLIANCE_COLOR == "blue"){
                Imgproc.rectangle(outPut,leftRect,rectColorBlue,2);
                Imgproc.rectangle(outPut, rightRect, rectColorBlue, 2);
                Imgproc.rectangle(outPut, middleRect, rectColorBlue, 2);
            } else if (Version1_OpMode.ALLIANCE_COLOR == "red"){
                Imgproc.rectangle(outPut,leftRect,rectColorRed,2);
                Imgproc.rectangle(outPut, rightRect, rectColorRed, 2);
                Imgproc.rectangle(outPut, middleRect, rectColorRed, 2);
            }
            
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


            if(leftavgfin > rightavgfin && leftavgfin > middleavgfin){
                telemetry.addLine("3");
                Version1_OpMode.propPosition = 1;
            }else if (leftavgfin < rightavgfin && rightavgfin > middleavgfin){
                telemetry.addLine("1");
                Version1_OpMode.propPosition = 3;
            }else{
                Version1_OpMode.propPosition = 2;
                telemetry.addLine("2");
            }

            return(outPut);
        }

    }
}
