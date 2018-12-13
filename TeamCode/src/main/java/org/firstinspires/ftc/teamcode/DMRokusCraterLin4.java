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


@Autonomous(name="DM Rokus Crater Linear v4", group="AutoLin")
//@Disabled
public class DMRokusCraterLin4 extends DMRokus_AbstractLin {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        //setup for sampling detector
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        //detector.enable();

        overrotate = false;

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
        telemetry.addData("Debug mode: ", debug);

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //Land the robot
        telemetry.addData("Step1", "Land the robot");    //
        telemetry.update();
        eLift(1,(5480/ENCODER_CNT_PER_IN_DRIVE),8);

        // rotate ~180 deg
        telemetry.addData("Step2", "Rotate ~180");    //
        telemetry.update();
        eDrive(0.5, (650/ENCODER_CNT_PER_IN_DRIVE),(-650/ENCODER_CNT_PER_IN_DRIVE),2);

        // Use DogeCV to get sampling order
        telemetry.addData("Step3", "Use DogeCV to get sampling order");    //
        telemetry.update();

        detector.enable();

        // reset the timeout time before starting
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 1.0) && !detector.isFound()) {
            Thread.yield();
        }

        telemetry.addData("Step3a - is found: ", detector.isFound());    //
        telemetry.update();

        if (!detector.isFound()) {
            eDrive(0.5, (-180/ENCODER_CNT_PER_IN_DRIVE),(180/ENCODER_CNT_PER_IN_DRIVE),1);
            overrotate = true;
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < 1.0) && !detector.isFound()) {
                Thread.yield();
            }
        }

        telemetry.addData("Step3a - over rotated: ", overrotate);

        if (detector.getAligned()) {
            telemetry.addData("Step3b", "Use DogeCV locate - found it");

            leftPos = false;
            centerPos = true;
            rightPos = false;
            telemetry.addData("Step3c", "Use DogeCV to get sampling order - found it -> CENTER");
            telemetry.update();
        } else {

            temp_align = detector.getXPosition() - detector.getAlignedx();
            if (temp_align > 100) {
                rightPos = false;
                if (!overrotate){
                    leftPos = true;
                    centerPos = false;
                    telemetry.addData("Step3c", "Use DogeCV to get sampling order - found it -> LEFT");
                } else {
                    leftPos = false;
                    centerPos = true;
                    telemetry.addData("Step3c", "Use DogeCV to get sampling order - found it -> CENTER");
                }
                telemetry.update();
            }

            if (temp_align < -100) {
                leftPos = false;
                centerPos = false;
                rightPos = true;
                telemetry.addData("Step3c", "Use DogeCV to get sampling order - found it -> RIGHT");
                telemetry.update();
            }

            runtime.reset();
            while ((abs(temp_align) > 100) &&
                    (runtime.seconds() < 5)) {

                if (temp_align != 0) {
                    targetPower = (float)(-0.0007*abs(temp_align));
                } else {
                    targetPower = -0.12f;
                }

                if (temp_align >100){
                    //telemetry.addData("move left"," wheel" );
                    //eDrive(0.5,-3.0,0,1000);
                    motorRight.setPower(targetPower);
                } else {
                    motorRight.setPower(0);
                }
                if (temp_align < -100){
                    //telemetry.addData("move right"," wheel" );
                    //eDrive(0.5,0,-3.0,1000);
                    motorLeft.setPower(targetPower);
                } else {
                    motorLeft.setPower(0);
                }
                sleep(100);
                temp_align = detector.getXPosition() - detector.getAlignedx();
                telemetry.addData("Debug - Position is: ", temp_align);
                telemetry.update();

            }
        }

        telemetry.update();

        // Turn off DogeCV
        detector.disable();

        // sample gold element
        telemetry.addData("Step4", "Select gold element");
        telemetry.update();

        targetDrDistInch = -26f; //default to center
        targetPower = DEFAULT_MOVE_SPEED;  // Set power

        if (leftPos)
        {
            targetDrDistInch = -30f; // Set target distance - left element
            telemetry.addData("Step4b", "Gold element at left position");
            telemetry.update();
            //eDrive(targetPower,-1.0,0,500);
        } else if (centerPos)
        {
            targetDrDistInch = -26f; // Set target distance - center element
            telemetry.addData("Step4b", "Gold element at center position");
            telemetry.update();
        } else if (rightPos)
        {
            targetDrDistInch = -30f; // Set target distance - right element
            telemetry.addData("Step4b", "Gold element at right position");
            telemetry.update();
            //eDrive(targetPower,0,-1.0,500);
        } else {
            telemetry.addData("Step4b", " - no element info... going with default");
            telemetry.update();
        }

        eDrive(targetPower,targetDrDistInch,targetDrDistInch,4);

        sleep(500);     // pause for motors to stop move

        // Park ~ 6"
        telemetry.addData("Step5", "Park at crater");
        telemetry.update();

        targetPower = DEFAULT_MOVE_SPEED;  // Set power
        targetDrDistInch = -3.5f; //default to center
        targetDrLeft = targetDrDistInch;
        targetDrRight = targetDrDistInch;

        if (leftPos)
        {
            telemetry.addData("Step5b", "Gold element at left position");
            telemetry.update();
            targetDrRight = targetDrDistInch - 2;
            //eDrive(targetPower,0,-1.0,0.5);
            //sleep(250);     // pause for motors to stop move
        } else if (centerPos)
        {
            telemetry.addData("Step5b", "Gold element at center position");
            telemetry.update();
        } else if (rightPos)
        {
            telemetry.addData("Step5b", "Gold element at right position");
            telemetry.update();
            targetDrLeft = targetDrDistInch -2;
            //eDrive(targetPower,-1.0,0,0.5);
            //sleep(250);     // pause for motors to stop move
        } else {
            telemetry.addData("Step5b", " - no element info... going with default");
            telemetry.update();
        }

        eDrive(targetPower,targetDrLeft,targetDrRight,1);

        //Done
        sleep(10000);     // pause for servos to move

        telemetry.addData("Step6", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void eDrive(double speed,
                       double leftInches, double rightInches,
                       double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * ENCODER_CNT_PER_IN_DRIVE);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * ENCODER_CNT_PER_IN_DRIVE);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
/*                telemetry.addData("Target - ",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Current Position - ",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                telemetry.update();
*/
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
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
/*                telemetry.addData("Target - ",  "Running to %7d", newTarget);
                telemetry.addData("Current Position - ",  "Running at %7d",
                        motorLift.getCurrentPosition());
                telemetry.update();
 */
            }

            // Stop all motion;
            motorLift.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
