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

@TeleOp(name = "TestMecanumEnc")
@Disabled
public class TestMecanumEncoder extends DMRelicAbstract {
    public TestMecanumEncoder() {
    }

    @Override
    public void init() {

        super.init();

        // Set all motors to run without encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);

        IncVal = 5;

    }

    @Override
    public void loop() {

        super.loop();
/*
        // Set drive motor power
        motorRightA.setPower(powerRightA);
        motorRightB.setPower(powerRightB);
        motorLeftA.setPower(powerLeftA);
        motorLeftB.setPower(powerLeftB);
*/
        targetPower = 0.5f;     // Set power
        targetDrRotateDeg = 0f;// Set rotation

        if (gamepad1.a) {
            targetDrDistInch = 12f; // Set target distance
            telemetry.addData("A pressed: ",  "12 inch");
            sleep(500);
        } else if (gamepad1.b) {
            targetDrDistInch = 6f; // Set target distance
            telemetry.addData("B pressed: ",  "6 inch");
            sleep(500);
        } else if (gamepad1.x) {
            targetDrDistInch = 2f; // Set target distance
            telemetry.addData("x pressed: ",  "2 inch");
            sleep(500);
        } else if (gamepad1.y) {
            targetDrDistInch = 1f; // Set target distance
            telemetry.addData("y pressed: ",  "1 inch");
            sleep(500);
        } else {
            targetDrDistInch = 0f; // Set target distance
            telemetry.addData("Please press a button",  "a - 12, b - 6, x - 2, y - 1");
            sleep(200);
        }




        targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
        targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
        targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
        targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

        telemetry.addData("RightA: ",  powerRightA);
        telemetry.addData("RightB: ",  powerRightB);
        telemetry.addData("LeftA: ",  powerLeftA);
        telemetry.addData("LeftB: ",  powerLeftB);
        telemetry.update();
// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}
