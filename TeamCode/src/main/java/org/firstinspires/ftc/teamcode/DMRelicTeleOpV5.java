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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;

@TeleOp(name = "DMRelicTeleOpV5")
@Disabled
public class DMRelicTeleOpV5 extends DMRelicAbstract {
    public DMRelicTeleOpV5() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorGlyphLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRelicArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Not using encoders for glyph lift
        //motorGlyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        tempposition = 0;

        snColor.enableLed(false);

        slowdown = false;
        drivepower = 1.0f;  //default drive speed

        //init glyph position
        sGlyphL.setPosition(0.1);
        sGlyphR.setPosition(0.28); //changed from 0.3

        //init back arm
        initbarm();

        //init relic jaw
        //sRelic.setPosition(0.47);
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


        //Controls for grabbing the glyph
        if (gamepad2.a) {
            if (Gopen) {
                spos = 27/180;
                sGlyphL.setPosition(0.4);  //Used to be 0.25. 0.28
                spos=133/180;
                sGlyphR.setPosition(0.14);  //Used to be 0.7.  0.18
                sleep(150);
                Gopen = false;
            }
            else {
                spos=10/180;
                sGlyphL.setPosition(0.09);  //changed from 0.09
                spos=145/180;
                sGlyphR.setPosition(0.28);  //changed from 0.3
                sleep(150);
                Gopen = true;
            }

        }


        if (gamepad2.b){
            sGlyphL.setPosition(0.4);
            sGlyphR.setPosition(0.14);
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
        if (gamepad2.left_stick_y < 0)  //((motorGlyphLift.getCurrentPosition() > -8500) && (gamepad2.left_stick_y < 0))
        {
            if (gamepad2.left_stick_y <= 0.1 && gamepad2.left_stick_y >= -0.1) {
                gamepad1.left_stick_y = 0;
            } else
            {
                throttleLift=gamepad2.left_stick_y;
                throttleLift = (float) scaleInput(throttleLift);
                motorGlyphLift.setPower(throttleLift);
                telemetry.addData("MGL move: ", throttleLift);
            }
            telemetry.addData("MGL power: ", throttleLift);

        } else if (gamepad2.left_stick_y > 0)  //((motorGlyphLift.getCurrentPosition() < -250) && (gamepad2.left_stick_y > 0))
        {
            if (gamepad2.left_stick_y <= 0.1 && gamepad2.left_stick_y >= -0.1) {
                gamepad1.left_stick_y = 0;
            } else
            {
                throttleLift=gamepad2.left_stick_y;
                throttleLift = (float) scaleInput(throttleLift);
                motorGlyphLift.setPower(throttleLift);
                telemetry.addData("MGL move: ", throttleLift);
            }
            telemetry.addData("MGL power: ", throttleLift);
        } else
        {
            motorGlyphLift.setPower(0);
        }

         //Open and close relic jaw
        if (gamepad2.x) {
            telemetry.addData("X Pressed, relicopen = ", relicopen);
            if (relicopen) {
                sRelicGrab.setPosition(0.56);  //Need to confirm position for close jaw - 100/180 = 0.62, changed to .56
                telemetry.addData("Close relic arm, relicopen = ", relicopen);
                sleep(150);
                relicopen = false;
            }
            else {
                sRelicGrab.setPosition(0.85);  //Need to confirm position for open jaw - 150/180 = 0.85
                telemetry.addData("Open relic arm, relicopen = ", relicopen);
                sleep(150);
                relicopen = true;
            }

        }

        //Extend and retrieve relic arm

        //temp_x = gamepad2.right_stick_x * -1;
        //temp_y = gamepad2.right_stick_y;
        targetRelicArmX = 30; //(int)(temp_x *5);
        //targetRelicArmY = (int)(temp_y *10);
        if (gamepad2.dpad_right)//(temp_x > 0)
        {

            motorRelicArm.setPower(1.0);
            tempposition = motorRelicArm.getCurrentPosition();
            telemetry.addData("relic arm up: ", tempposition);
            /*
            //relicarmforward(targetRelicArmX,(float) 1.0);
            telemetry.addData("mRA: ", targetRelicArmX);
            sleep(120);
            */

        } else if (gamepad2.dpad_left)//(temp_x < 0)
        {
            motorRelicArm.setPower(-1.0);
            tempposition = motorRelicArm.getCurrentPosition();
            telemetry.addData("relic arm down: ", tempposition);
            /*
            relicarmback(targetRelicArmX,(float) 1.0);
            telemetry.addData("mRA: ", -targetRelicArmX);
            sleep(120);
            */
        } else
        {
            motorRelicArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRelicArm.setTargetPosition(tempposition);
            motorRelicArm.setPower(1.0);
            telemetry.addData("relic arm stop: ", tempposition);
        }

        //lower and raise the relic arm extension
        if(gamepad2.dpad_up)
        {
            rarmDown();
            telemetry.addData("Relic Arm up: ", rarmTime);
        }
        else if (gamepad2.dpad_down)
        {
            rarmUp();
            telemetry.addData("Relic Arm down: ", rarmTime);
        }
        else {
            rarmStop();
            telemetry.addData("Relic Arm stop: ", rarmTime);
        }
        /*
        //Lower and raise relic arm
        if (temp_y > 0)
        {


            srelicarmforward(temp_y);
            telemetry.addData("SRA: ", temp_y);

        } else if (temp_y < 0)  //((motorGlyphLift.getCurrentPosition() < -250) && (gamepad2.left_stick_y > 0))
        {
            srelicarmback(temp_y);
            telemetry.addData("SRA: ", temp_y);
        }
        */


       // telemetry.addData("MRE position: ", motorRelicExtend.getCurrentPosition());
        telemetry.addData("MRA position: ", motorRelicArm.getCurrentPosition());
        telemetry.addData("Target Relic Arm: ", targetRelicArmX);
        telemetry.addData("GP2-LX: ", gamepad2.right_stick_x);
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
