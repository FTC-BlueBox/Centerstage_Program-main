
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
    private DcMotor MOTOR_LEFT_LINEARRACK,MOTOR_RIGHT_LINEARRACK, MOTOR_INTAKE;
    private Servo HOLDER_ROTATE, CLAMP1, PLANE_LAUNCH, CLAMP2, AUTOHOLDER;


    @Override
    public void runOpMode() {

        double MotorPower = 0.0;
        double reduceSpeedFactor = 0.4;                          // reduce motor power
        double intakeMotorPower = -0.8;
        int intakeStatus = 1;                                    //check if intake is running (binary)

        int linearRackHomePos = 0;
        int linearRackHighPos = 2500;
        int linearRackTarget = linearRackHomePos;
        double holderHomePos = 0.17;
        double holderFlippedPos = 0.5;
        double holderPos = holderHomePos;
        double planeLaunchPos = 0.2;
        double planeHoldPos = 0.6;
        double planePos = planeHoldPos;
        double clamp1ClosePos = 1;
        double clamp2ClosePos = 0.9;
        double clampOpenPos = 0.0;
        double clamp1Pos = clampOpenPos;
        double clamp2Pos = clampOpenPos;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Initialize all hardware variables in the hardwaremap
        MOTOR1  = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3  = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        MOTOR_LEFT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-LEFT-LINEARRACK");
        MOTOR_RIGHT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-RIGHT-LINEARRACK");
        MOTOR_INTAKE = hardwareMap.get(DcMotor.class, "MOTOR-INTAKE");

        HOLDER_ROTATE = hardwareMap.get(Servo.class, "HOLDER-ROTATE");
        CLAMP1 = hardwareMap.get(Servo.class, "CLAMP1");
        CLAMP2 = hardwareMap.get(Servo.class, "CLAMP2");
        PLANE_LAUNCH = hardwareMap.get(Servo.class, "PLANE-LAUNCH");
        AUTOHOLDER = hardwareMap.get(Servo.class, "AUTOHOLDER");


        // Reset Encoders
        MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Bring servos to start position
        HOLDER_ROTATE.setPosition(holderPos);   //main arm flip
        CLAMP1.setPosition(clamp1Pos);          //front pixel clamp
        CLAMP2.setPosition(clamp2Pos);          //back pixel clamp
        PLANE_LAUNCH.setPosition(planeHoldPos); //plane launch hold position

        String ALLIANCE_COLOR = "";

        //  Wait for the game to start (driver presses PLAY)
        while(!opModeIsActive())
        {
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
            MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        //  run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            // gamepad 1
            if (gamepad1.right_stick_y != 0)                            // Robot Movement: Forward and Backward
            {
                MotorPower = gamepad1.right_stick_y * reduceSpeedFactor;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(-MotorPower);
            }
            else if (gamepad1.right_stick_x != 0 )                      // Robot Movement: Turning
            {
                MotorPower =  gamepad1.right_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);
            }
             else if (gamepad1.left_stick_x != 0 )                       // Robot Movement: Strafing
            {
                MotorPower = gamepad1.left_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);

            }
             else
            {
                MotorPower =  0.0;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            }

             if(gamepad1.y){                                                        //Linear Rack up and down movement
                 if(linearRackTarget == linearRackHomePos){
                     MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
                      MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);
                     linearRackTarget = linearRackHighPos;
                 }
                 else if(linearRackTarget == linearRackHighPos){
                     MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHomePos);
                     MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHomePos);
                     linearRackTarget = linearRackHomePos;
                 }
                 MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                 MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                 MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                 if (MOTOR_LEFT_LINEARRACK.isBusy())  MOTOR_LEFT_LINEARRACK.setPower(-1);
                 else                                 MOTOR_LEFT_LINEARRACK.setPower(0);

                 if (MOTOR_RIGHT_LINEARRACK.isBusy())  MOTOR_RIGHT_LINEARRACK.setPower(1);
                 else                                  MOTOR_RIGHT_LINEARRACK.setPower(0);

                 sleep(1000);

             }
             if(gamepad1.a){
                 if(holderPos == holderHomePos){
                     holderPos = holderFlippedPos;
                 } else{
                     holderPos = holderHomePos;
                 }
                 HOLDER_ROTATE.setPosition(holderPos);
                 sleep(200);
             }
             if(gamepad1.b){
                 if(intakeStatus == 1){
                     MOTOR_INTAKE.setPower(0.0);
                     intakeStatus = 0;
                 }else{
                     MOTOR_INTAKE.setPower(intakeMotorPower);
                     intakeStatus = 1;
                 }
                 sleep(200);
             }

             if(gamepad1.x){                                    //Launch Plane
                 if(planePos == planeHoldPos){
                     planePos = planeLaunchPos;
                 } else {
                     planePos = planeHoldPos;
                 }
                 PLANE_LAUNCH.setPosition(planePos);
                 sleep(200);
             }
             if(gamepad1.right_bumper){                        //Open/Close clamp 1
                 if (clamp1Pos == clampOpenPos) {
                     CLAMP1.setPosition(clamp1ClosePos);
                     clamp1Pos = clamp1ClosePos;
                 }else{
                     CLAMP1.setPosition(clampOpenPos);
                     clamp1Pos = clampOpenPos;
                 }
                 sleep(200);
             }
            if(gamepad1.left_bumper){                        //Open/Close clamp 2
                if (clamp2Pos == clampOpenPos) {
                    CLAMP2.setPosition(clamp2ClosePos);
                    clamp2Pos = clamp2ClosePos;
                }else{
                    CLAMP2.setPosition(clampOpenPos);
                    clamp2Pos = clampOpenPos;
                }
                sleep(200);
            }

            telemetry.update();
        }
    }
}