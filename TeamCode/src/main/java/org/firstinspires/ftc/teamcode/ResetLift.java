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

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;


@Autonomous(name="Reset Lift", group="Temp")
//@Disabled
public class ResetLift extends DMRokus_AbstractLin {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        //set motors to use encoders
        motorLeft = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);

        motorRight = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorArm = hardwareMap.dcMotor.get(MOTOR_ARM);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        motorBox = hardwareMap.dcMotor.get(MOTOR_BOX);
        motorBox.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBox.setDirection(DcMotor.Direction.REVERSE);

        motorLift = hardwareMap.dcMotor.get(MOTOR_LIFT);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotor.Direction.REVERSE);

        motorExtend = hardwareMap.dcMotor.get(MOTOR_EXTENDER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtend.setDirection(DcMotor.Direction.REVERSE);

        //turn off auto clear for telemetry
        telemetry.setAutoClear(false);

        telemetry.addData("I am alive - ","init");

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //Land the robot
        telemetry.addData("Step1", "reset lift arm");    //
        telemetry.update();
        eLift(1,(-6500/ENCODER_CNT_PER_IN_DRIVE),11);

        //Done
        sleep(1000);     // pause for servos to move

        telemetry.addData("Step6", "Complete");
        telemetry.update();
    }


    public void eLift(double speed,
                       double Inches, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = motorLift.getCurrentPosition() + (int)(Inches * ENCODER_CNT_PER_IN_DRIVE);
            motorLift.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLift.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target - ",  "Running to %7d", newTarget);
                telemetry.addData("Current Position - ",  "Running at %7d",
                        motorLift.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLift.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
