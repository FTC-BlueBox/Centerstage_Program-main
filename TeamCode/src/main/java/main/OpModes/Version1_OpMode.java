
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

        package main.OpModes;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode V1", group="Linear OpMode")
public class Version1_OpMode extends LinearOpMode {


    private DcMotor MOTOR1, MOTOR2, MOTOR3, MOTOR4;
    private DcMotor MOTOR_LINEARRACK, MOTOR_INTAKE;
    private Servo PLANE_SERVO, OUTTAKE_ARM, PIXEL_HOLDER, ARM_JOINT;

    double  reduceSpeedFactor = 0.6;                       // reduce motor power
    double intakeMotorPower = 0;                            // intake motor power
    int  linearRackHomePosition  = 0; // check               // linear rack home position (constant)
    int linearRackPos = linearRackHomePosition;                                  // linear rack set position
    int  linearRackHighPosition  = 200;                     // linear rack high position (constant)
    double armHomePosition = 0.0;                           // arm home position (constant)
    double armFLippedPosition = -1.2;                        // arm flipped position (constant)
    double armPosition = armHomePosition;                    // set arm position
    double MotorPower = 0;                                  // originally start motor at 0
    double readyPlanePosition = 0.5;                        //pre-launch plane position
    double afterLaunchPosition = 1.0;                       //Post-launch plane position
    double planePosition = readyPlanePosition;              //Set plane position
    double pixelReleasePosition = 0.0;                      //Dropped door position
    double pixelHeldPosition = 0.0;                         //Closed door position for pixel intake
    double pixelPosition = pixelHeldPosition;
    double armJoinHomePosition = 0.1;
    double armJointPosition = armJoinHomePosition;
    double armJointFLippedPosition = 0.4;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Initialize all hardware variables in the hardwaremap
        MOTOR1  = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3  = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        MOTOR_LINEARRACK  = hardwareMap.get(DcMotor.class, "LINEARRACK");
        MOTOR_INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        PLANE_SERVO = hardwareMap.get(Servo.class, "PLANE");
        OUTTAKE_ARM = hardwareMap.get(Servo.class, "OUTTAKE-ARM");
        PIXEL_HOLDER = hardwareMap.get(Servo.class, "PIXEL-HOLDER");
        ARM_JOINT = hardwareMap.get(Servo.class, "ARM-JOINT");

        // Set zero power behavior to brake
        MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders
        MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        String ALLIANCE_COLOR = "";

        //  Wait for the game to start (driver presses PLAY)
        while(!opModeIsActive())
        {
            // start up arm to starting position
            OUTTAKE_ARM.setPosition(armPosition);
            PLANE_SERVO.setPosition(readyPlanePosition);
            PIXEL_HOLDER.setPosition(pixelPosition);


            //  Alliance Color (X=Blue, B=Red)
            if      (gamepad1.x || gamepad2.x)  ALLIANCE_COLOR = "BLUE";
            else if (gamepad1.b || gamepad2.b)  ALLIANCE_COLOR = "RED";

            telemetry.addData("Alliance", ALLIANCE_COLOR);
            telemetry.update();

            waitForStart();                                                     //  Wait for the Play Button press
        }
        if (opModeIsActive()) {
            // Turn on encoders
            MOTOR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        // do this

        }
        //  run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // gamepad 1 (is there overlap with controllers)
            if (gamepad1.right_stick_y != 0 || gamepad2.right_stick_y != 0)      // Robot Movement: Forward and Backward
            {
                MotorPower = gamepad1.right_stick_y * reduceSpeedFactor;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);
            }
            else if (gamepad1.right_stick_x != 0 || gamepad2.right_stick_x != 0)  // Robot Movement: Turning
            {
                MotorPower =  gamepad1.right_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);
            }
            else if (gamepad1.left_stick_x != 0 || gamepad2.left_stick_x != 0)      // Robot Movement: Strafing  (incorrect)
            {
                MotorPower =  gamepad1.left_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(MotorPower);
            } else{

                MotorPower =  0;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            }
            if (gamepad1.right_bumper) {                                                       // Bring up/down linear rack
                if (linearRackPos == linearRackHomePosition)
                {
                    linearRackPos = linearRackHighPosition;
                }
                else{
                    linearRackPos = linearRackHomePosition;
                }
                MOTOR_LINEARRACK.setTargetPosition(linearRackPos);
            }
           /* if(gamepad1.x){//bty
                if(armPosition == armHomePosition){
                    armPosition = armFLippedPosition;
                } else {
                    armPosition = armHomePosition;
                }
                OUTTAKE_ARM.setPosition(armPosition);
            }*/
            if(gamepad1.a){
                if(armJointPosition == armJoinHomePosition){
                    armJointPosition = armJointFLippedPosition;
                }else{
                    armJointPosition = armJoinHomePosition;
                }
                ARM_JOINT.setPosition(armJointPosition);
            }


            if(gamepad2.b){
                if(pixelPosition == pixelHeldPosition){
                    pixelPosition = pixelReleasePosition;
                } else{
                    pixelPosition = pixelHeldPosition;
                }
                PIXEL_HOLDER.setPosition(pixelPosition);
            }

            // gamepad 2
            if (gamepad2.x) {                                                      // start/stop intake
                if (intakeMotorPower == 0){
                    intakeMotorPower = 0.4;
                }
                else {
                    intakeMotorPower = 0;
                }
                MOTOR_INTAKE.setPower(intakeMotorPower);
            }

            if(gamepad1.y){
                if (planePosition == readyPlanePosition){                         //Launch paper Airplane
                    planePosition = afterLaunchPosition;
                } else {
                    planePosition = readyPlanePosition;
                }
                PLANE_SERVO.setPosition(planePosition);
            }



            telemetry.update();
        }
    }
}