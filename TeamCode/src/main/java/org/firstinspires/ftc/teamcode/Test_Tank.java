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
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Test_Tank")
//@Disabled
public class Test_Tank extends DMRokus_Abstract {
    public Test_Tank() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        scaledrive = false;
        single = false;


    }

    @Override
    public void loop() {

        //super.loop();

        double left;
        double right;

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
                if (gamepad1.left_stick_y <= 0.1 && gamepad1.left_stick_y >= -0.1) {
                    gamepad1.left_stick_y = 0;
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

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("scaledrive is: ", scaledrive);
        telemetry.addData("Single is: ", single);
        //telemetry.update();

// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}