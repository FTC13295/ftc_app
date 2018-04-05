/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;

@TeleOp(name = "DMRelicTeleOp")
@Disabled
public class DMRelicTeleOp extends DMRelicAbstract {
    public DMRelicTeleOp() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorGlyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //fieldOrient = false;
        //bDirection = true;

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
        glyphL=0;
        glyphR=180;
        IncVal = 5;
        Gdown = false;
        Gopen = true;

        snColor.enableLed(false);

        slowdown = false;
        drivepower = 1.0f;  //default drive speed

        //init glyph position
        sGlyphL.setPosition(0.1);
        sGlyphR.setPosition(0.9); //changed from 0.8 - changed from 0.75

        //init back arm
        initbarm();

    }

    @Override
    public void loop() {

        super.loop();

        // Set drive motor power
        motorRightA.setPower(powerRightA);
        motorRightB.setPower(powerRightB);
        motorLeftA.setPower(powerLeftA);
        motorLeftB.setPower(powerLeftB);

        // Set controls
        velocityDrive = -gamepad1.left_stick_y; //
        strafeDrive = -gamepad1.left_stick_x;  //change to -
        rotationDrive = -gamepad1.right_stick_x;  //change to -

       // Scale drive motor power for better control at low power
        powerRightA = (float) scaleInput(powerRightA);
        powerRightB = (float) scaleInput(powerRightB);
        powerLeftA = (float) scaleInput(powerLeftA);
        powerLeftB = (float) scaleInput(powerLeftB);

        //Create dead-zone for drive train controls
        if (gamepad1.left_stick_x <= 0.05 && gamepad1.left_stick_x >= -0.05) {
            gamepad1.left_stick_x = 0;
        }

        if (gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05) {
            gamepad1.left_stick_y = 0;
        }

        if (gamepad1.right_stick_x <= 0.05 && gamepad1.right_stick_x >= -0.05) {
            gamepad1.right_stick_x = 0;
        }

        // If the left stick and the right stick is used it halves the power  of the motors for better accuracy
        if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0) {


            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        } else if (gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 && gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0) {

            powerRightA = Range.clip(powerRightA, -0.5f, 0.5f);
            powerRightB = Range.clip(powerRightB, -0.5f, 0.5f);
            powerLeftA = Range.clip(powerLeftA, -0.5f, 0.5f);
            powerLeftB = Range.clip(powerLeftB, -0.5f, 0.5f);

        } else {

/*
            if (gamepad1.x ) {
                if (slowdown) {
                    slowdown = false;
                    drivepower = 1.0f;
                    telemetry.addData("Changed to ", "Regular speed");
                } else {
                    slowdown = true;
                    drivepower = PowerRatio;
                    telemetry.addData("Changed to ", "Half speed");
                }
                sleep(200);
            } else {
                if (slowdown) {
                    drivepower = PowerRatio;
                    telemetry.addData("Driving at ", "half speed");
                } else {
                    slowdown = true;
                    drivepower = 1.0f;
                    telemetry.addData("Driving at ", "full speed");
                }
            }
*/

            if (gamepad1.right_bumper)
            {
                drivepower = SLOW_POWER;
                telemetry.addData("Driving at slow speed - right bumper pushed - override: ", drivepower);
            } else if (gamepad1.left_bumper)
            {
                drivepower = FULL_POWER;
                telemetry.addData("Driving at slow speed - right bumper pushed - override: ", drivepower);
            } else
            {
                drivepower = REG_POWER;
                telemetry.addData("Driving at regular speed - no bumper used: ", drivepower);
            }

            powerRightA = Range.clip(powerRightA, -drivepower, drivepower);
            powerRightB = Range.clip(powerRightB, -drivepower, drivepower);
            powerLeftA = Range.clip(powerLeftA, -drivepower, drivepower);
            powerLeftB = Range.clip(powerLeftB, -drivepower, drivepower);


        }


        powerRightA = (velocityDrive + rotationDrive + strafeDrive) * drivepower;
        powerRightB = (velocityDrive + rotationDrive - strafeDrive) * drivepower;
        powerLeftA = (velocityDrive - rotationDrive - strafeDrive) * drivepower;
        powerLeftB = (velocityDrive - rotationDrive + strafeDrive) * drivepower;
        /*
        //Change direction that is front for the robot

            if (gamepad1.dpad_left) {
                bDirection = true; // Arm is front.
            }
            if (gamepad1.dpad_right) {
                bDirection = false; // Collection is front
            }

            if (fieldOrient) {
                bDirection = true;
            }

            if (bDirection) // Glyph is front
            {
                powerRightA = velocityDrive + rotationDrive + strafeDrive;
                powerRightB = velocityDrive + rotationDrive - strafeDrive;
                powerLeftA = velocityDrive - rotationDrive - strafeDrive;
                powerLeftB = velocityDrive - rotationDrive + strafeDrive;
            } else  // Relic is front
            {
                powerRightA = velocityDrive - rotationDrive - strafeDrive;
                powerRightB = velocityDrive - rotationDrive + strafeDrive;
                powerLeftA = velocityDrive + rotationDrive + strafeDrive;
                powerLeftB = velocityDrive + rotationDrive - strafeDrive;
            }
*/

        //Control Gem arm

            if (gamepad2.b) {
                if (!Gdown) {
                    spos=135/180;
                    sGem.setPosition(0.8);
                    sleep(150);
                    Gdown = true;
                }
                else {
                    sGem.setPosition(0);
                    sleep(150);
                    Gdown = false;
                }
            }
        //Test Color/Distance sensor
        /*
            if (Gdown) {
                telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", sensorColor.alpha());
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                if (sensorColor.red() > sensorColor.blue()) {
                    telemetry.addData("Gem ", "RED");
                }
                else {
                    telemetry.addData("Gem ", "BLUE");
                }
            }
         */

        //Controls for grabbing the glyph
        if (gamepad2.a) {
            if (Gopen) {
                spos = 27/180;
                sGlyphL.setPosition(0.2);  //Used to be 0.16
                spos=133/180;
                sGlyphR.setPosition(0.8);  //Used to be 0.72 - added 0.15 to 0.65
                sleep(150);
                Gopen = false;
            }
            else {
                spos=10/180;
                sGlyphL.setPosition(0.09);  //changed from 0.05
                spos=145/180;
                sGlyphR.setPosition(0.9);  //changed from 0.83 - added 0.15 to 0.75
                sleep(150);
                Gopen = true;
            }

        }

        // lower the back arm
        if (gamepad2.y && !bArmDown)
        {
            bArmDown = true;
            lowerbarm(20);
            sleep(250);
        }

        //Back Arm testing...
        if(gamepad2.right_bumper)
        {
            barmDown();
            telemetry.addData("Back Arm  ", "down");
        }
        else if (gamepad2.left_bumper)
        {
            barmUp();
            telemetry.addData("Back Arm  ", "up");
        }
        else {
            barmStop();
            telemetry.addData("Back Arm  ", "stop");
        }

        // Glyph Lift operations
        /*
        if(gamepad2.dpad_down)
        {
            gliftDown();
            telemetry.addData("Glyph Lift ", "down");
        }
        else if (gamepad2.dpad_up)
        {
            gliftUp();
            telemetry.addData("Glyph Lift ", "up");
        }
        else {
            gliftStop();
            telemetry.addData("Glyph Lift ", "stop");
        }
*/
        //testing Glyph Lift.....
        /*
        if(gamepad2.x)
        {
            //sGLift2.setDirection(Servo.Direction.REVERSE);
            //sGLift2.setPosition(1);
            sGLift2.setPosition(1.0);
        }
        else if(gamepad2.y)
        {
            //sGLift2.setDirection(Servo.Direction.FORWARD);
            //sGLift2.setPosition(1);
            sGLift2.setPosition(0.0);
        }
        else {
            sGLift2.setPosition(0.5);
        }
        */

        //testing with dpad for position of servos
/*
            if (gamepad2.dpad_right)
            {
                glyphL = glyphL + IncVal;
                sGlyphL.setPosition(glyphL/180);
                sleep(100);

            }

            if (gamepad2.dpad_left)
            {
                if (glyphL > 0)
                {
                    glyphL = glyphL - IncVal;
                }
                else
                {
                    glyphL = 0;
                }
                sGlyphL.setPosition(glyphL/180);
                sleep(100);

            }
            if (gamepad2.dpad_up)
            {
                if (glyphR < 180)
                {
                    glyphR = glyphR + IncVal;
                }
                else
                {
                    glyphR = 180;
                }
                sGlyphR.setPosition(glyphR/180);
                sleep(100);

            }

            if (gamepad2.dpad_down)
            {
                glyphR = glyphR - IncVal;
                sGlyphR.setPosition(glyphR/180);
                sleep(100);

            }
*/
        /*


            if (gamepad2.x && !grabbed)
            {
                grabbed = true;
            }
            if (gamepad2.x && grabbed)
            {
                grabbed = false;
            }

            if (grabbed) {
                servoGlyph1.setPosition(90);
                servoGlyph2.setPosition(90);
            }
            if (!grabbed) {
                servoGlyph1.setPosition(45);
                servoGlyph2.setPosition(45);
            }

*/

        //Controls for lifting the glyph
        /*

            //Set controls for lift
            throttleLift = gamepad2.left_stick_y;

            // Clip and scale the throttle, and then set motor power.
            throttleLift = Range.clip(throttleLift, -1, 1);
            throttleLift = (float) scaleInput(throttleLift);
            motorGlyphLift.setPower(throttleLift);

            //Create dead-zone for lift control
            if (gamepad2.left_stick_y <= 0.05 && gamepad2.left_stick_y >= -0.05) {
                gamepad2.left_stick_y = 0;
            }

        */

        telemetry.addData("Driving at full speed: ", slowdown);
        telemetry.update();
// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}
