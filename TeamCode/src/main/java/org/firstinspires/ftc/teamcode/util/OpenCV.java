package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//IMPORT INTO CONTROL HUB!!

@Autonomous
public class OpenCV extends OpMode{
    
    OpenCvWebcam webcam = null;

    @Override
    public void init(){

        WebcamName webcamName = hardwareMap.get(WebcameName.class,"webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new examplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened(){
                webcam.startStreaming(1080,720,OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){
            }
        });
    }

    @Override
    public void loop(){

    }

    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);

        public Mat processFram(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1,1,539,719);
            Rect rightRect = new Rect(539,1,539,719);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColor,2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

            if(leftavgfin > rightavgfin){
                telemetry.addLine("left");
            }else{
                telemetry.addLine("right")
            }

            return(outPut);
        }
    }
}
