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

import static android.os.SystemClock.sleep;

@TeleOp(name = "ServoTest")
public class ServoTest extends DMRelicAbstract {
    public ServoTest() {
    }

    @Override
    public void init() {

        super.init();

        glyphL = 0;
        IncVal = 5;
        glyphR = 180;
        telemetry.addData("use dpad left and right to change servo position by 5, ", "use dpad up and down to change servo position by 1 (pad2)");
        telemetry.addData("Change mode from % to number by pressing x", "");
        telemetry.update();
    }

    @Override
    public void loop() {

        super.loop();

        //use x to change mode
        if (gamepad2.x)
        {
            if (glyphR == 180)
            {
                glyphR = 1;
            } else
            {
                glyphR = 180;
            }
        }

        //testing with dpad for position of servos
        telemetry.addData("use dpad left and right to change servo position by 5, ", "use dpad up and down to change servo position by 1, ");

        if (gamepad2.dpad_right)
        {
            glyphL = glyphL + IncVal;
            if (glyphL >180)
            {
                glyphL = 180;
            }
            sRelic.setPosition(glyphL/glyphR);
            sleep(150);
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
             sRelic.setPosition(glyphL/glyphR);
             sleep(150);
        }
        if (gamepad2.dpad_up)
        {
            glyphL = glyphL + 1;
            if (glyphL >180)
            {
                glyphL = 180;
            }
            sRelic.setPosition(glyphL/glyphR);
            sleep(150);
        }
        if (gamepad2.dpad_down)
        {
            if (glyphL > 0)
            {
                glyphL = glyphL - 1;
            }
            else
            {
                glyphL = 0;
            }
            sRelic.setPosition(glyphL/glyphR);
            sleep(150);

        }
        telemetry.addData("Relic servo position: ", glyphL);
        telemetry.addData("Relic servo position used in program (press x to change mode): ", glyphL/glyphR);
        telemetry.update();
// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}
