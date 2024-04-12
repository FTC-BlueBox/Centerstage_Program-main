
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
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        @TeleOp(name="Drive Only", group="Linear OpMode")
        public class driveOnly extends LinearOpMode {

            private DcMotor MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR_INTAKE;
            private CRServo GROUND_WHEELS_MOTOR;
            private Servo  INTAKE_LIFT_MOTOR, CLAMP1, CLAMP2, BOX_FLIP;

            @Override
            public void runOpMode() {

                double MotorPower = 0.0;
                double reduceSpeedFactor = 0.85;                          // Reduce motor power (controller b only)


                telemetry.addData("Status", "Initialized");
                telemetry.update();

                //  Initialize all hardware variables in the hardware-map
                MOTOR1 = hardwareMap.get(DcMotor.class, "MOTOR1");
                MOTOR2 = hardwareMap.get(DcMotor.class, "MOTOR2");
                MOTOR3 = hardwareMap.get(DcMotor.class, "MOTOR3");
                MOTOR4 = hardwareMap.get(DcMotor.class, "MOTOR4");
                MOTOR_INTAKE = hardwareMap.get(DcMotor.class, "MOTOR-INTAKE");

                INTAKE_LIFT_MOTOR = hardwareMap.get(Servo.class, "INTAKE-LIFT");
                GROUND_WHEELS_MOTOR = hardwareMap.get(CRServo.class, "GROUND-WHEELS");
                CLAMP1 = hardwareMap.get(Servo.class, "CLAMP1");
                CLAMP2 = hardwareMap.get(Servo.class, "CLAMP2");
                BOX_FLIP = hardwareMap.get(Servo.class, " BOX-FLIP");

                // Reset DC Motor encoders
                MOTOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                MOTOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                MOTOR3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                MOTOR4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                MOTOR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MOTOR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MOTOR3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MOTOR4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                MOTOR1.setDirection(DcMotorSimple.Direction.REVERSE);
                MOTOR3.setDirection(DcMotorSimple.Direction.REVERSE);

                CLAMP1.setPosition(0.5); //0 is home
                CLAMP2.setPosition(0.5);
                BOX_FLIP.setPosition(0.92); //flipped position - 0.1

                INTAKE_LIFT_MOTOR.setPosition(1);

                waitForStart();                         //  Wait for the Play Button press

                if (opModeIsActive()) {

                    MOTOR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           // Turn on encoders
                    MOTOR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    MOTOR3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    MOTOR4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



                }

                while (opModeIsActive())                                           // Run until driver presses STOP
                {

                    GROUND_WHEELS_MOTOR.setPower(1); //faster?
                        if (gamepad1.right_stick_y != 0)                           // Robot Movement: Forward and Backward
                        {
                            MotorPower = gamepad1.right_stick_y * reduceSpeedFactor;                   // Game-pad one runs at full speed

                            MOTOR1.setPower(MotorPower);
                            MOTOR2.setPower(MotorPower);
                            MOTOR3.setPower(MotorPower);
                            MOTOR4.setPower(MotorPower);
                        } else if (gamepad1.right_stick_x != 0)                      // Robot Movement: Turning
                        {
                            MotorPower = gamepad1.right_stick_x * reduceSpeedFactor;

                            MOTOR1.setPower(-MotorPower);
                            MOTOR2.setPower(MotorPower);
                            MOTOR3.setPower(-MotorPower);
                            MOTOR4.setPower(MotorPower);

                        } else if (gamepad1.left_stick_x != 0)                       // Robot Movement: Strafing
                        {
                            MotorPower = gamepad1.left_stick_x * reduceSpeedFactor;

                            MOTOR1.setPower(MotorPower);
                            MOTOR2.setPower(-MotorPower);
                            MOTOR3.setPower(-MotorPower);
                            MOTOR4.setPower(MotorPower);

                        } else {
                            MotorPower = 0.0;                                       // Default power to zero

                            MOTOR1.setPower(MotorPower);
                            MOTOR2.setPower(MotorPower);
                            MOTOR3.setPower(MotorPower);
                            MOTOR4.setPower(MotorPower);
                        }


                    }

                    // Game-pad 1: LR is front, semi-auto scoring, releases both clamps, control LR and intake flip
                    // Game-pad 2: Intake is front, turns on intake, closes/opens clamps, lifts intake box from drag, hang, plane launch

                    // Game-pad2


                return 0;
            }
        }