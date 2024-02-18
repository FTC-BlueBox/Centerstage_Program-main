
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
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.hardware.Servo;
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
            private DcMotor MOTOR_LEFT_LINEARRACK,MOTOR_RIGHT_LINEARRACK, MOTOR_INTAKE;
            private Servo HOLDER_ROTATE, CLAMP1, PLANE_LAUNCH, CLAMP2, AUTOHOLDER;
            private DigitalChannel LED1, LED2;

            @Override
            public void runOpMode() {

                double MotorPower = 0.0;
                double reduceSpeedFactor = 0.85;                          // Reduce motor power (controller b only)
                double intakeMotorPower = -1;
                int intakeStatus = 1;                                     // Check if intake is running (binary)

                int linearRackHomePos = 0;                                // Linear rack preset positions
                int linearRackHighPos = 3600;
                int linearRackTarget = linearRackHomePos;
                int linearRackLiftPos = 2700;

                double holderHomePos = 0.16;                             // Top intake rotator positions
                double holderFlippedPos = 0.52;
                double holderPos = holderHomePos;

                double planeLaunchPos = 0.2;                              // Plane launch servo positions
                double planeHoldPos = 0.6;
                double planePos = planeHoldPos;

                double clamp1ClosePos = 0.85;                                // Pixel clamp positions
                double clamp2ClosePos = 0.9;
                double clampOpenPos = 0.5;
                double clamp1Pos = clamp1ClosePos;
                double clamp2Pos = clamp2ClosePos;

                telemetry.addData("Status", "Initialized");
                telemetry.update();

                //  Initialize all hardware variables in the hardware-map
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

                // Init LED's and set state
                LED1 = hardwareMap.get(DigitalChannel.class, "LIGHT");
                LED2 = hardwareMap.get(DigitalChannel.class, "LIGHT2");
                LED1.setMode(DigitalChannel.Mode.OUTPUT);
                LED1.setState(true);
                LED2.setMode(DigitalChannel.Mode.OUTPUT);
                LED2.setState(true);

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
                HOLDER_ROTATE.setPosition(holderPos);   // Main arm flip to home position
                CLAMP1.setPosition(clamp1Pos);          // Front and back pixel clamps up
                CLAMP2.setPosition(clamp2Pos);
                PLANE_LAUNCH.setPosition(planeHoldPos); // Plane launch to hold position
                AUTOHOLDER.setPosition(1);              // Lift auto holder out of the way

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
                    if (gamepad2.right_stick_y == 0 && gamepad2.right_stick_x == 0 && gamepad2.left_stick_x == 0) {  // Prevent controller conflicts between game-pads
                        if (gamepad1.right_stick_y != 0)                           // Robot Movement: Forward and Backward
                        {
                            MotorPower = gamepad1.right_stick_y * reduceSpeedFactor;                   // Game-pad one runs at full speed

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
                            MotorPower = gamepad1.left_stick_x * reduceSpeedFactor;

                            MOTOR1.setPower(-MotorPower);
                            MOTOR2.setPower(-MotorPower);
                            MOTOR3.setPower(MotorPower);
                            MOTOR4.setPower(MotorPower);

                        } else {
                            MotorPower = 0.0;                                       // Default power to zero

                            MOTOR1.setPower(MotorPower);
                            MOTOR2.setPower(MotorPower);
                            MOTOR3.setPower(MotorPower);
                            MOTOR4.setPower(MotorPower);
                        }
                    } else if (gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0) {  // Prevent controller conflicts between game-pads
                        if (gamepad2.right_stick_y != 0)                           // Robot Movement: Forward and Backward (flipped)
                        {
                            MotorPower = -gamepad2.right_stick_y * reduceSpeedFactor;

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

                            MOTOR1.setPower(-MotorPower);
                            MOTOR2.setPower(-MotorPower);
                            MOTOR3.setPower(MotorPower);
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
                    if (gamepad1.y) {                                                      //Linear rack manual up and down movement
                        if (linearRackTarget == linearRackHomePos) {
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);
                            linearRackTarget = linearRackHighPos;
                        } else if (linearRackTarget == linearRackHighPos) {
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHomePos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHomePos);
                            linearRackTarget = linearRackHomePos;
                        }
                        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LEFT_LINEARRACK.isBusy()) MOTOR_LEFT_LINEARRACK.setPower(-1);
                        else MOTOR_LEFT_LINEARRACK.setPower(0);

                        if (MOTOR_RIGHT_LINEARRACK.isBusy()) MOTOR_RIGHT_LINEARRACK.setPower(1);
                        else MOTOR_RIGHT_LINEARRACK.setPower(0);

                        sleep(1300);
                    }
                    if (gamepad1.a) {                                                      // Manually flip intake
                        if (holderPos == holderHomePos) {
                            holderPos = holderFlippedPos;
                        } else {
                            holderPos = holderHomePos;
                        }
                        HOLDER_ROTATE.setPosition(holderPos);
                        sleep(200);
                    }
                    if (gamepad1.right_bumper) {                                           // Semi-auto to score
                        if (linearRackTarget == linearRackHomePos) {                        // First bring up linear rack
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHighPos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHighPos);
                            linearRackTarget = linearRackHighPos;
                        } else if (linearRackTarget == linearRackHighPos) {
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHomePos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHomePos);
                            linearRackTarget = linearRackHomePos;
                            holderPos = holderHomePos;                                     // Flip box before coming down
                            HOLDER_ROTATE.setPosition(holderPos - 0.06);
                            clamp1Pos = clamp1ClosePos;
                            clamp2Pos = clamp2ClosePos;
                            CLAMP1.setPosition(clamp1Pos);
                            CLAMP2.setPosition(clamp2Pos);
                            sleep(200);
                        }
                        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LEFT_LINEARRACK.isBusy()) MOTOR_LEFT_LINEARRACK.setPower(-1);
                        else MOTOR_LEFT_LINEARRACK.setPower(0);

                        if (MOTOR_RIGHT_LINEARRACK.isBusy()) MOTOR_RIGHT_LINEARRACK.setPower(1);
                        else MOTOR_RIGHT_LINEARRACK.setPower(0);

                        sleep(1500);
                        if (linearRackTarget == linearRackHighPos) {                          // Flip box at the top
                            holderPos = holderFlippedPos;
                            HOLDER_ROTATE.setPosition(holderPos);
                            sleep(200);
                        }

                    }
                    if (gamepad1.left_bumper) {                                               // Semi-auto scoring pixels part 2
                        if (clamp1Pos == clampOpenPos) {                                    // Open both clamps (with delay)
                            CLAMP1.setPosition(clamp1ClosePos);
                            clamp1Pos = clamp1ClosePos;
                        } else {
                            CLAMP1.setPosition(clampOpenPos);
                            clamp1Pos = clampOpenPos;
                        }
                        sleep(1000);
                        if (clamp2Pos == clampOpenPos) {
                            CLAMP2.setPosition(clamp2ClosePos);
                            clamp2Pos = clamp2ClosePos;
                        } else {
                            CLAMP2.setPosition(clampOpenPos);
                            clamp2Pos = clampOpenPos;
                        }
                        sleep(400);
                    }
                    if (gamepad1.right_trigger >= 0.5) {                                      // Bring up linear rack very low (stuck pixel)
                        if (linearRackTarget == linearRackHomePos) {
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-500);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(500);
                            linearRackTarget = linearRackHighPos;
                        } else if (linearRackTarget == linearRackHighPos) {
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHomePos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHomePos);
                            linearRackTarget = linearRackHomePos;
                        }
                        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LEFT_LINEARRACK.isBusy()) MOTOR_LEFT_LINEARRACK.setPower(-1);
                        else MOTOR_LEFT_LINEARRACK.setPower(0);

                        if (MOTOR_RIGHT_LINEARRACK.isBusy()) MOTOR_RIGHT_LINEARRACK.setPower(1);
                        else MOTOR_RIGHT_LINEARRACK.setPower(0);

                        sleep(400);
                    }


                    // Game-pad 2
                    if (gamepad2.a) {                                                            // Turn on intake
                        if (intakeStatus == 1) {
                            MOTOR_INTAKE.setPower(0.0);
                            HOLDER_ROTATE.setPosition(holderHomePos - 0.06);
                            LED2.setState(true);
                            LED1.setState(true);     // Turn on LED indicator
                            intakeStatus = 0;
                        } else if (linearRackTarget == linearRackHomePos && holderPos == holderHomePos) {
                            MOTOR_INTAKE.setPower(intakeMotorPower);
                            HOLDER_ROTATE.setPosition(holderHomePos);
                            LED2.setState(false);
                            LED1.setState(false);
                            intakeStatus = 1;
                        }
                        sleep(200);

                    }
                    if (gamepad2.y) {                                                             // Bring up LR to hang height
                        if (linearRackTarget == linearRackHomePos) {
                            linearRackTarget = linearRackHighPos;
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackLiftPos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackLiftPos);
                        } else if (linearRackTarget == linearRackHighPos) {
                            MOTOR_LEFT_LINEARRACK.setTargetPosition(-linearRackHomePos);
                            MOTOR_RIGHT_LINEARRACK.setTargetPosition(linearRackHomePos);
                            linearRackTarget = linearRackHomePos;
                        }
                        MOTOR_LEFT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        MOTOR_RIGHT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        MOTOR_LEFT_LINEARRACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (MOTOR_LEFT_LINEARRACK.isBusy()) MOTOR_LEFT_LINEARRACK.setPower(-1);
                        else MOTOR_LEFT_LINEARRACK.setPower(0);

                        if (MOTOR_RIGHT_LINEARRACK.isBusy()) MOTOR_RIGHT_LINEARRACK.setPower(1);
                        else MOTOR_RIGHT_LINEARRACK.setPower(0);

                        sleep(1000);
                    }
                    if (gamepad2.b) {                                                             // Close/Open both clamps and lift intake off the ground
                        if (clamp1Pos == clampOpenPos) {
                            CLAMP1.setPosition(clamp1ClosePos);
                            clamp1Pos = clamp1ClosePos;
                        } else {
                            CLAMP1.setPosition(clampOpenPos);
                            clamp1Pos = clampOpenPos;
                            if (holderPos == holderHomePos) {
                                HOLDER_ROTATE.setPosition(holderHomePos);
                            }
                        }
                        if (clamp2Pos == clampOpenPos) {
                            CLAMP2.setPosition(clamp2ClosePos);
                            clamp2Pos = clamp2ClosePos;
                        } else {
                            CLAMP2.setPosition(clampOpenPos);
                            clamp2Pos = clampOpenPos;
                        }
                        sleep(200);
                    }
                    if (gamepad2.right_bumper) {                                                 // Open/Close clamp 1
                        if (clamp1Pos == clampOpenPos) {
                            CLAMP1.setPosition(clamp1ClosePos);
                            clamp1Pos = clamp1ClosePos;
                        } else {
                            CLAMP1.setPosition(clampOpenPos);
                            clamp1Pos = clampOpenPos;
                        }
                        sleep(200);
                    }
                    if (gamepad2.left_bumper) {                                                  // Open/Close clamp 2
                        if (clamp2Pos == clampOpenPos) {
                            CLAMP2.setPosition(clamp2ClosePos);
                            clamp2Pos = clamp2ClosePos;
                        } else {


                            CLAMP2.setPosition(clampOpenPos);
                            clamp2Pos = clampOpenPos;
                        }
                        sleep(200);
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
                            intakeStatus = 0;
                        } else {
                            MOTOR_INTAKE.setPower(-intakeMotorPower);
                            intakeStatus = 1;
                        }
                        sleep(200);

                    }


                    // Logic and LED's
                    if(clamp1Pos == clamp1ClosePos && clamp2Pos == clamp2ClosePos && linearRackTarget == linearRackHomePos) {
                        HOLDER_ROTATE.setPosition(holderPos - 0.06);                         // Lift intake when driving
                    }

                    telemetry.update();
                }
        }
        }