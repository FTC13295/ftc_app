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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/* ------------------------------------------------------------------
 * This Op Mode is a template for Autonomous Control
 *
 * Control Modules
 *  1. Motor Controller Module
 *  2. Motor Controller Module
 *
 * Gamepads
 * 	Not used; however, for troubleshooting prior to competition, Gamepad1's "A" button will allow
 * 	the programmer to advance the sequence 1 step.
 * ------------------------------------------------------------------
 */
@Autonomous(name = "ShootClose", group = "RiderModes")
@Disabled
public class CyberShootClose extends CyberVortexAbstract
{
    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    @Override
    public void init() {

        super.init();


    }

    @Override
    public void loop()
    {

        super.loop();

        // START ROBOT SEQUENCE
        // Establish the robot sequence of operation with the Switch operation.
        // The ab ctive Case (i.e., sequence step) is established by the value in seqRobot.
        // After a Case executes, Break the switch to prevent executing subsequent cases unintentionally.
        switch (seqRobot)
        {

            case 1: // Transition all motors to Run-to-Position mode.
            {
                motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorCollect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                seqRobot++;
                break;  // Note: The Break command will ensure one scan of code executed while motor
                // modes take effect. If Run-to-Position not enabled, unexpected operations
                // may occur.
            }

            case 2: // Move robot forward 33" at 50% speed
            {
                // Define drive train target position and motor power.
                targetDrRotateDeg = 0f; // Not used in this step, but reported via telemetry in the next step.
                targetDrDistInch = 33f; // Set target distance
                targetPower = 0.5d;  // Set power

                // Use this OpModes's custom cmdMoveR method to calculate new target (in encoder
                // counts) and to initiate the move. cmdMoveR initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeft);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRight);

                // Jump to next step to wait for the commanded moves to complete
                seqRobot++;
                break; // Break allows one scan of code prior to checking motor progress.
            }


            case 3: // Hold until drive train move is complete
            {
                // Use this OpModes's custom chkMove to determine if motor move(s) are complete
                // chkMove Parameters (motor, target, allowed +/- error counts from target)
                // May need to add motor-is-busy check to ensure electric breaking complete.
                // May need to compensate for motor power if one motor is faster than another to keep straight line.
                if (chkMove(motorLeft, targetPosLeft, ERROR_DRV_POS) &&
                        chkMove(motorRight, targetPosRight, ERROR_DRV_POS))
                {    // If drive train at target, hold position for X amount of time to stabilize motors.
                    if (!pulseCaseMoveDone)
                    {    // Restart the runtime clock when the motors reach target
                        resetStartTime();
                        pulseCaseMoveDone = true;
                    }
                    if (getRuntime() >= DELAY_DRV_MOV_DONE)
                    {    // Advance the sequence once the drive train has been at target for X amount of time.
                        if (seqRobot == 3)
                        {
                            seqRobot++;
                        }
                        else
                        {
                            seqRobot = 99;
                        }
                    }
                }


                break; // Note: The Break command will add one code scan after position found
            }

            case 4:
            case 8:
            case 10:
            case 12:// Reset encoders
            {
                motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeft.setTargetPosition(0);
                motorRight.setTargetPosition(0);
                seqRobot++;
                break;
            }


            case 5:// Turn at 25% power
            {
                motorLeft.setTargetPosition(ROTATE_COUNTS);
                motorRight.setTargetPosition(-ROTATE_COUNTS);
                motorLeft.setPower(.25);
                motorRight.setPower(.25);
                seqRobot++;
                break;
            }

            case 6:// Check to see if the turn was up to standards
            {
                if (motorLeft.getCurrentPosition() >= ROTATE_COUNTS_MIN && motorLeft.getCurrentPosition() <= ROTATE_COUNTS_MAX &&
                        motorRight.getCurrentPosition() <= -ROTATE_COUNTS_MIN && motorRight.getCurrentPosition() >= -ROTATE_COUNTS_MAX)
                {
                    seqRobot++;
                    break;
                }
                else
                {
                    seqRobot = 5;
                    break;
                }
            }

            case 7:
            case 11:// Wait and shoot particle
            {
                if(seqRobot == 7)
                {
                    slp(500);
                }

                if(seqRobot == 11)
                {
                    slp(1000);
                }

                int rotateCounts = motorFlicker.getCurrentPosition() + LAUNCH_ROTATE;
                motorFlicker.setPower(1);
                motorFlicker.setTargetPosition(rotateCounts);
                slp(1000);

                seqRobot++;
                break;
            }

            case 9:// Reload particle into flicker
            {
                long inTime = System.currentTimeMillis() + COLLECT_RUN_TIME;

                while (System.currentTimeMillis() < inTime)
                {
                    motorCollect.setPower(1);
                }
                motorCollect.setPower(0);

                seqRobot++;
                break;
            }

            case 99:  // Done
            {
                break;
            }

            default:
            {
                break;
            }
            // NOTE - By stringing many cases together a complex operation can be built.

        }    // End Robot Sequence

        telemetry.addData("1. ", "I am alive");
        telemetry.addData("2. ", "Seq #  : " + seqRobot);
        telemetry.addData("3. ", "Channel Left mode = " + motorLeft.getMode());
        telemetry.addData("4. ", "Drive Pos L/R" + motorLeft.getCurrentPosition() + "/" + motorRight.getCurrentPosition());
        telemetry.addData("5a. ", "Minimum goal: " + ROTATE_COUNTS);
        telemetry.addData("5b. ", "Maximum goal: " + ROTATE_COUNTS_MAX);
        telemetry.addData("Color Values","");
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Sat", hsvValues[1]);
        telemetry.addData("V", hsvValues[2]);
        telemetry.addData("Current color Blue?", blue);
        telemetry.addData("Current color Red?", red);
        telemetry.addData("Current color White?", white);
        telemetry.addData("White line set?", whiteLine);
        telemetry.addData("Runtime", getRuntime());
        telemetry.update();
    } // End OpMode Loop Method

    private void slp(int slptime) {
        try {
            sleep(slptime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

} // End OpMode
