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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

@TeleOp(name = "DMRelicTeleOpV2")
public class DMRelicTeleOpV2 extends DMRelicAbstract {
    public DMRelicTeleOpV2() {
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
        if (gamepad1.left_stick_x <= 0.1 && gamepad1.left_stick_x >= -0.1) {
            gamepad1.left_stick_x = 0;
            strafeDrive = 0;
        }

        if (gamepad1.left_stick_y <= 0.1 && gamepad1.left_stick_y >= -0.1) {
            gamepad1.left_stick_y = 0;
            velocityDrive = 0;
        }

        if (gamepad1.right_stick_x <= 0.1 && gamepad1.right_stick_x >= -0.1) {
            gamepad1.right_stick_x = 0;
            rotationDrive = 0;
        }

        // If the left stick is used choose direction with most force
        temp_x_stick = gamepad1.left_stick_x;
        temp_y_stick = gamepad1.left_stick_y;
        if (temp_x_stick <0) {
            temp_x_stick *= -1;
        }
        if (temp_y_stick <0) {
            temp_y_stick *= -1;
        }
        if (temp_x_stick > temp_y_stick) {  // Strafing
            velocityDrive = 0;
            rotationDrive = 0;
        } else {  //  driving (no strafe)
            strafeDrive = 0;
        }

        if (gamepad1.right_bumper) {
            drivepower = SLOW_POWER;
            telemetry.addData("Driving at slow speed - right bumper pushed - override: ", drivepower);
        } else if (gamepad1.left_bumper) {
            drivepower = FULL_POWER;
            telemetry.addData("Driving at slow speed - right bumper pushed - override: ", drivepower);
        } else {
            drivepower = REG_POWER;
            telemetry.addData("Driving at regular speed - no bumper used: ", drivepower);
        }

        powerRightA = (velocityDrive + rotationDrive + strafeDrive) * drivepower;
        powerRightB = (velocityDrive + rotationDrive - strafeDrive) * drivepower;
        powerLeftA = (velocityDrive - rotationDrive - strafeDrive) * drivepower;
        powerLeftB = (velocityDrive - rotationDrive + strafeDrive) * drivepower;


        //Control Gem arm
/*
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

        if (gamepad2.b){
            sGlyphL.setPosition(0.4);
            sGlyphR.setPosition(0.5);
        }

/*
        // lower the back arm
        if (gamepad2.y && !bArmDown)
        {
            bArmDown = true;
            lowerbarm(20);
            sleep(250);
        }
*/

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
