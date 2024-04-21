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
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import main.color_sensor;
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

@TeleOp(name="Main OpMode", group="Linear OpMode")
public class Main_OpMode extends LinearOpMode {

    private DcMotor MOTOR1, MOTOR2, MOTOR3, MOTOR4;
    private DcMotor MOTOR_LEFT_LINEARRACK, MOTOR_RIGHT_LINEARRACK, MOTOR_INTAKE;
    private Servo BOX_FLIP, CLAMP1, PLANE_LAUNCH, CLAMP2, INTAKE_LIFT;
    private CRServo GROUND_WHEELS;
    private DigitalChannel LED1G, LED1R, LED2G, LED2R;

    ColorSensor colorSensorFront;    // Hardware Device Object
    ColorSensor colorSensorBack;

    @Override
    public void runOpMode() {

        //color_sensor colorSensor = new color_sensor();
        //float hue = colorSensor.frontSensorHue;

        double MotorPower = 0.0;
        double reduceSpeedFactor = 1;                          // Reduce motor power (controller b only)
        double intakeMotorPower = 50;
        int intakeStatus = 0;                                     // Check if intake is running (binary)

        int linearRackHomePos = 45;                                // Linear rack preset positions
        int linearRackMiddlePos = 1500;
        int linearRackHighPos = 3500;
        boolean highMode = false;
        int linearRackTarget = linearRackHomePos;
        int linearRackLiftPos = 2500;

        double boxHomePos = 1;                             // Top intake rotator positions
        double boxFlippedPos = 0.39;
        double boxPos = boxHomePos;

        // !!
        double planeLaunchPos = 0.6;                              // Plane launch servo positions
        double planeHoldPos = 0.2;
        double planePos = planeHoldPos;

        double clamp1ClosePos = 0.35;                                // Pixel clamp positions
        double clamp1OpenPos = 0.0;
        double clamp2OpenPos = 0.1;
        double clamp2ClosePos = 0.4;
        double clamp1Pos = clamp1OpenPos;
        double clamp2Pos = clamp2OpenPos;

        double intakePosLow = 0.79; //lowest
        double intakePosHigh = 0.93; //highest
        double intakePos = intakePosLow;

        //camera
        float hsvValuesFront[] = {0F,0F,0F};
        float hsvValuesBack[] = {0F,0F,0F};

        final float valuesFront[] = hsvValuesFront;
        final float valuesBack[] = hsvValuesBack;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  Initialize all hardware variables in the hardware-map
        MOTOR1 = hardwareMap.get(DcMotor.class, "MOTOR1");
        MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
        MOTOR3 = hardwareMap.get(DcMotor.class, "MOTOR3");
        MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
        MOTOR_LEFT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-LEFT-LINEARRACK");
        MOTOR_RIGHT_LINEARRACK = hardwareMap.get(DcMotor.class, "MOTOR-RIGHT-LINEARRACK");
        MOTOR_INTAKE = hardwareMap.get(DcMotor.class, "MOTOR-INTAKE");
        BOX_FLIP = hardwareMap.get(Servo.class, "BOX-FLIP");
        CLAMP1 = hardwareMap.get(Servo.class, "CLAMP2");
        CLAMP2 = hardwareMap.get(Servo.class, "CLAMP1");
        INTAKE_LIFT = hardwareMap.get(Servo.class, "INTAKE-LIFT");
        PLANE_LAUNCH = hardwareMap.get(Servo.class, "PLANE-LAUNCH");
        GROUND_WHEELS = hardwareMap.get(CRServo.class, "GROUND-WHEELS");

        colorSensorBack = hardwareMap.get(ColorSensor.class, "front-color");
        colorSensorFront= hardwareMap.get(ColorSensor.class, "back-color");

        // Set the LED in the beginning
        colorSensorFront.enableLed(true);
        colorSensorBack.enableLed(true);


        // Init LED's and set state
        LED1R = hardwareMap.get(DigitalChannel.class, "LIGHT1R");
        LED2R = hardwareMap.get(DigitalChannel.class, "LIGHT2R");
        LED1R.setMode(DigitalChannel.Mode.OUTPUT);
        LED1R.setState(true);
        LED2R.setMode(DigitalChannel.Mode.OUTPUT);
        LED2R.setState(true);

        LED1G = hardwareMap.get(DigitalChannel.class, "LIGHT1G");
        LED2G = hardwareMap.get(DigitalChannel.class, "LIGHT2G");
        LED1G.setMode(DigitalChannel.Mode.OUTPUT);
        LED1G.setState(true);
        LED2G.setMode(DigitalChannel.Mode.OUTPUT);
        LED2G.setState(true);

        // Reset DC Motor encoders
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
        BOX_FLIP.setPosition(boxPos);   // Main arm flip to home position
        CLAMP1.setPosition(clamp1Pos);          // Front and back pixel clamps up
        CLAMP2.setPosition(clamp2Pos);
        PLANE_LAUNCH.setPosition(planeHoldPos); // Plane launch to hold position
        INTAKE_LIFT.setPosition(1);

        waitForStart();                         //  Wait for the Play Button press

        if (opModeIsActive()) {

            MOTOR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           // Turn on encoders
            MOTOR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MOTOR3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MOTOR4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        while (opModeIsActive())                                           // Run until driver presses STOP
        {

            Color.RGBToHSV(colorSensorFront.red() * 8, colorSensorFront.green() * 8, colorSensorFront.blue() * 8, hsvValuesFront);
            Color.RGBToHSV(colorSensorBack.red() * 8, colorSensorBack.green() * 8, colorSensorBack.blue() * 8, hsvValuesBack);

            telemetry.addData("color front: ", hsvValuesBack[0]);
            telemetry.addData("color back: ", hsvValuesFront[0]);
            telemetry.update();


            if (gamepad2.right_stick_y == 0 && gamepad2.right_stick_x == 0 && gamepad2.left_stick_x == 0) {  // Prevent controller conflicts between game-pads
            if (gamepad1.right_stick_y != 0)                           // Robot Movement: Forward and Backward (flipped)
            {
                MotorPower = -gamepad1.right_stick_y * reduceSpeedFactor;

                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(MotorPower);
            } else if (gamepad1.right_stick_x != 0)                      // Robot Movement: Turning
            {
                MotorPower = gamepad1.right_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(-MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(-MotorPower);
            } else if (gamepad1.left_stick_x != 0)                       // Robot Movement: Strafing
            {
                MotorPower = -gamepad1.left_stick_x * reduceSpeedFactor;

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(-MotorPower);
                MOTOR3.setPower(-MotorPower);
                MOTOR4.setPower(MotorPower);

            } else {
                MotorPower = 0.0;                                      // Default power to zero

                MOTOR1.setPower(MotorPower);
                MOTOR2.setPower(MotorPower);
                MOTOR3.setPower(MotorPower);
                MOTOR4.setPower(MotorPower);
            }
            } else if (gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0) {  // Prevent controller conflicts between game-pads

                     if (gamepad2.right_stick_y != 0)                           // Robot Movement: Forward and Backward (flipped)
                     {
                         MotorPower = gamepad2.right_stick_y * reduceSpeedFactor;

                         MOTOR1.setPower(-MotorPower);
                         MOTOR2.setPower(MotorPower);
                         MOTOR3.setPower(-MotorPower);
                         MOTOR4.setPower(MotorPower);
                     } else if (gamepad2.right_stick_x != 0)                      // Robot Movement: Turning
                     {
                         MotorPower = gamepad2.right_stick_x * reduceSpeedFactor;

                         MOTOR1.setPower(-MotorPower);
                         MOTOR2.setPower(-MotorPower);
                         MOTOR3.setPower(-MotorPower);
                         MOTOR4.setPower(-MotorPower);
                     } else if (gamepad2.left_stick_x != 0)                       // Robot Movement: Strafing
                     {
                         MotorPower = gamepad2.left_stick_x * reduceSpeedFactor;

                         MOTOR1.setPower(MotorPower);
                         MOTOR2.setPower(-MotorPower);
                         MOTOR3.setPower(-MotorPower);
                         MOTOR4.setPower(MotorPower);

                     } else {
                         MotorPower = 0.0;                                      // Default power to zero

                         MOTOR1.setPower(MotorPower);
                         MOTOR2.setPower(MotorPower);
                         MOTOR3.setPower(MotorPower);
                         MOTOR4.setPower(MotorPower);
                     }
                 }



            // Game-pad 1: LR is front, semi-auto scoring, releases both clamps, control LR and intake flip
            // Game-pad 2: Intake is front, turns on intake, closes/opens clamps, lifts intake box from drag, hang, plane launch

            // Game-pad2

            if (gamepad1.right_bumper) {                                           // Semi-auto to score
                if (linearRackTarget == linearRackHomePos) {                        // First bring up linear rack
                    if(highMode){ linearRackTarget = linearRackHighPos; }             //check for high or middle mode
                    else { linearRackTarget = linearRackMiddlePos; }

                } else if (linearRackTarget == linearRackHighPos || linearRackTarget == linearRackMiddlePos) {
                    linearRackTarget = linearRackHomePos;
                    boxPos = boxHomePos;                                     // Flip box before coming down
                    BOX_FLIP.setPosition(boxPos);
                    clamp1Pos = clamp1OpenPos;
                    clamp2Pos = clamp2OpenPos;
                    CLAMP1.setPosition(clamp1Pos);
                    CLAMP2.setPosition(clamp2Pos);
                    sleep(400);
                }
                moveLinearRack(linearRackTarget);

                if (linearRackTarget == linearRackHighPos || linearRackTarget == linearRackMiddlePos) {                         // Flip box at the top
                    sleep(1000);
                    boxPos = boxFlippedPos;
                    BOX_FLIP.setPosition(boxPos);
                    sleep(200);
                }

            }
            if (gamepad1.left_bumper) {

                if (clamp2Pos == clamp2OpenPos) {
                    CLAMP2.setPosition(clamp2ClosePos);
                    clamp2Pos = clamp2ClosePos;
                } else {
                    CLAMP2.setPosition(clamp2OpenPos);
                    clamp2Pos = clamp2OpenPos;
                }
                sleep(800);
                // Semi-auto scoring pixels part 2
                if (clamp1Pos == clamp1OpenPos) {                                    // Open both clamps (with delay)
                    CLAMP1.setPosition(clamp1ClosePos);
                    clamp1Pos = clamp1ClosePos;
                } else {
                    CLAMP1.setPosition(clamp1OpenPos);
                    LED1R.setState(false);
                    LED2R.setState(false);
                    LED1G.setState(false);
                    LED2G.setState(false);
                    clamp1Pos = clamp1OpenPos;
                }
                sleep(200);
            }
            if(gamepad1.x){
                if(highMode == true){
                    highMode = false;
                } else{
                    highMode = true;
                }
                //highMode = !highMode;
                LED1R.setState(true);
                LED2R.setState(false);
                LED1G.setState(false);
                LED2G.setState(true);
            }

            // Game-pad 2
            if (gamepad2.a) {                                                            // Turn on intake
                if (intakeStatus == 1) {
                    MOTOR_INTAKE.setPower(0.0);
                    GROUND_WHEELS.setPower(0.0);
                    LED1R.setState(false);
                    LED2R.setState(false);
                    // Turn on LED indicator
                    intakeStatus = 0;

                } else if (intakeStatus == 0 && linearRackTarget == linearRackHomePos) {
                    moveLinearRack(linearRackHomePos);
                    MOTOR_INTAKE.setPower(intakeMotorPower);
                    GROUND_WHEELS.setPower(1);
                    LED1R.setState(true);
                    LED2R.setState(true);
                    intakeStatus = 1;
                }
                sleep(200);

            }
            //turn off intake and close clamps
            if (hsvValuesBack[0] < 215 && intakeStatus == 1) {
                CLAMP2.setPosition(clamp2ClosePos);
                clamp2Pos = clamp2ClosePos;
                LED1G.setState(false);
                LED2G.setState(false);
            }
           /* if (hsvValuesFront[0] < 215  && clamp2Pos == clamp2ClosePos && intakeStatus == 1) {                                      // auto close pixels
                LED1.setState(false);
                LED2.setState(false);
            }*/
            if(gamepad2.b){
                CLAMP1.setPosition(clamp1ClosePos);
                clamp1Pos = clamp1ClosePos;
                MOTOR_INTAKE.setPower(0);
                GROUND_WHEELS.setPower(0);
                intakeStatus = 0;
            }

            if (gamepad2.y) {                                                             // Bring up LR to hang height
                if (linearRackTarget == linearRackHomePos) {
                    if(highMode) linearRackTarget = linearRackHighPos;
                    else linearRackTarget = linearRackMiddlePos;
                    moveLinearRack(linearRackLiftPos);

                } else if (linearRackTarget == linearRackHighPos || linearRackTarget == linearRackMiddlePos ) {
                    linearRackTarget = linearRackHomePos;
                    moveLinearRack(linearRackTarget);
                }
            }

            if (gamepad2.x) {                                                           // Launch Plane
                if (planePos == planeHoldPos) {
                    planePos = planeLaunchPos;
                } else {
                    planePos = planeHoldPos;
                }
                 PLANE_LAUNCH.setPosition(planePos);
                sleep(200);
            }
            if (gamepad2.right_trigger >= 0.5) {                                       // Reverse intake
                if (intakeStatus == 1) {
                    MOTOR_INTAKE.setPower(0.0);
                    GROUND_WHEELS.setPower(0.0);
                    intakeStatus = 0;
                } else {
                    MOTOR_INTAKE.setPower(-intakeMotorPower);
                    GROUND_WHEELS.setPower(-1);
                    intakeStatus = 1;
                }
                sleep(200);

            }
            if (gamepad2.dpad_down) {                                                   //move intake up and down
                if (intakePos > intakePosLow) {
                    intakePos = intakePos - 0.005;
                }
            }
            if(intakePos < intakePosHigh) {
                if (gamepad2.dpad_up) {
                    intakePos = intakePos + 0.005;
                }
            }
            if(gamepad2.dpad_right){
                intakePos = intakePosLow;
            }
            if(gamepad2.dpad_left){
                intakePos = intakePosHigh;
            }
            INTAKE_LIFT.setPosition(intakePos);

            if (intakePos == intakePosLow && intakeStatus == 1){
                INTAKE_LIFT.setPosition(0.763); //low!!
            }
            if (clamp1Pos == clamp1ClosePos && clamp2Pos == clamp2ClosePos && intakePos == intakePosLow){
                INTAKE_LIFT.setPosition(intakePos);
            }
        }
    }
    public void moveLinearRack(int height){
        MOTOR_LEFT_LINEARRACK.setTargetPosition(height);
        MOTOR_RIGHT_LINEARRACK.setTargetPosition(-height);

        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (MOTOR_LEFT_LINEARRACK.isBusy()) MOTOR_LEFT_LINEARRACK.setPower(-1);
        else MOTOR_LEFT_LINEARRACK.setPower(0);

        if (MOTOR_RIGHT_LINEARRACK.isBusy()) MOTOR_RIGHT_LINEARRACK.setPower(1);
        else MOTOR_RIGHT_LINEARRACK.setPower(0);
    }
}

//TODO: test pixel color readings, find correct intake heights1
// , x indicator,