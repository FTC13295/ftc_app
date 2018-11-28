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


@TeleOp(name = "Test_Motors")
//@Disabled
public class Test_Motors extends DMRokus_Abstract {
    public Test_Motors() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scaledrive = true;
        single = true;

        //set encoders on for arm
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetpos = HD_MOTOR_ENC * 65/360;
    }

    @Override
    public void loop() {

        //super.loop();

        double left, right, arm, extend;

        if (gamepad1.a)
        {
            if (scaledrive)
            {
                scaledrive = false;
            } else
            {
                scaledrive = true;
            }
        }

        if(gamepad1.b) {
            motorArm.setTargetPosition(targetpos);
            motorArm.setPower(1);
        }

        if (gamepad1.x){
            if (targetpos < 130) {
                targetpos = targetpos + 10;
            } else {
                targetpos = 140;
            }
        }

        if (gamepad1.y){
            if (targetpos > 10) {
                targetpos = targetpos - 10;
            } else {
                targetpos = 0;
            }
        }

  /*
        if (gamepad1.b)
        {
            if (single)
            {
                single = false;
            } else
            {
                single = true;
            }
        }
*/

        if(!single)
        {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }
        else
        {
            left = -gamepad1.right_stick_y + gamepad1.right_stick_x;
            right = -gamepad1.right_stick_y - gamepad1.right_stick_x;

            if (left  > 1)
            {
                left = 1;
            } else if (left < -1)
            {
                left = -1;
            }
            if (right  > 1)
            {
                right = 1;
            } else if (right < -1)
            {
                right = -1;
            }
        }

        if (scaledrive)
        {
            // Scale drive motor power for better control at low power
            left = (float) scaleInput(left);
            right = (float) scaleInput(right);

            if (!single) {
                //Create dead-zone for drive train controls
                if (gamepad1.right_stick_x <= 0.1 && gamepad1.right_stick_x >= -0.1) {
                    gamepad1.right_stick_x = 0;
                    left = 0;
                }

                if (gamepad1.right_stick_y <= 0.1 && gamepad1.right_stick_y >= -0.1) {
                    gamepad1.right_stick_y = 0;
                    right = 0;
                }
            }
        }



        motorLeft.setPower(left);
        motorRight.setPower(right);

        if (gamepad1.right_trigger > 0.2){
            motorBox.setPower(1.0);
        }else if (gamepad1.left_trigger > 0.2){
            motorBox.setPower(-1.0);
        }else {
            motorBox.setPower(0);
        }

        if (gamepad1.dpad_up){
            motorLift.setPower(1.0);
        } else if(gamepad1.dpad_down) {
            motorLift.setPower(-1.0);
        } else {
            motorLift.setPower(0);
        }

        extend = gamepad1.left_stick_x;
        arm = gamepad1.left_stick_y;
        //Create dead-zonef
        if (gamepad1.left_stick_x <= 0.2 && gamepad1.left_stick_x >= -0.2) {
            gamepad1.left_stick_x = 0;
            extend = 0;
        }

        if (gamepad1.left_stick_y <= 0.2 && gamepad1.left_stick_y >= -0.2) {
            gamepad1.left_stick_y = 0;
            arm = 0;
        }
        if (arm >0){
            motorArm.setPower(1.0);
        } else if (arm <0){
            motorArm.setPower(-1.0);
        } else {
            motorArm.setPower(0);
        }

        motorExtend.setPower(extend);

        // Send telemetry message to signify robot running;
        //telemetry.addData("Press a to switch scaledrive, press b to switch single mode","");
        telemetry.addData("Left stick drive, right stick v - arm rotation, h - extend","");
        telemetry.addData("Right Trigger for collector, pad v - lift","");
        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
        telemetry.addData("targetpos", targetpos);
        telemetry.addData("position", motorArm.getCurrentPosition());
        telemetry.addData("scaledrive is: ", scaledrive);
        telemetry.addData("Single is: ", single);
        telemetry.addData("Arm",  "%.2f", arm);
        telemetry.addData("Extend", "%.2f", extend);
        //telemetry.update();

// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}
