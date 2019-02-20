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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

@Autonomous(name="DM Rokus Crater Linear 2.333 go to depo", group="AutoLin")
//@Disabled
public class DMRokusCraterLin2_3333 extends DMRokus_AbstractLin {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        float tangle, tempratio, tempspeed;

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

        //Init IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.useExternalCrystal = true;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.setMsTransmissionInterval(100);

        //turn off auto clear for telemetry
        telemetry.setAutoClear(false);

        telemetry.addData("I am alive - ", "init");
        telemetry.addData("Debug mode: ", debug);

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //get initial angle
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //Land the robot
        telemetry.addData("Step1", "Land the robot");    //
        telemetry.update();

        eLift(1.0, (4270 / ENCODER_CNT_PER_IN_DRIVE), 10);

        //landing angle
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Landing angle: ", angles.firstAngle);
        telemetry.update();


        //pause for 0.1 sec
        sleep(100);

        // rotate ~180 deg
        telemetry.addData("Step2", "Rotate ~180");    //
        telemetry.update();

        temp_angle = angles.firstAngle;
        turnPower = 1.0f;
        targetPower=1.0f;

        tangle = 178;
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        while (temp_angle > -178) { //reduced from -175 to 170 due to overturning
            tempratio = (float) (abs(tangle + temp_angle) / 180);
            tempspeed = (float) turnPower * tempratio + 0.2f;
            motorLeft.setPower(tempspeed);
            motorRight.setPower(tempspeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            temp_angle = angles.firstAngle;
        }
        telemetry.addData("exit while, angle: ", angles.firstAngle);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Stopped motors", " Done");
        telemetry.update();

        //pause for 0.1 sec
        sleep(100);

        // Get closer
        eDrive(1, -6.5, -6.5, 5);

        //get current angle
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        temp_angle = angles.firstAngle;

        //determine turn direction
        if (temp_angle > 0) {
            motorRight.setDirection(DcMotor.Direction.REVERSE);
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            turncw = false;
        } else {
            motorRight.setDirection(DcMotor.Direction.FORWARD);
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
            turncw = true;
        }

        //turn to 180
        telemetry.addData("Rotate to 180 - angle: ", angles.firstAngle);
        while (abs(temp_angle) < 178) {
            motorLeft.setPower(turnPower);
            motorRight.setPower(turnPower);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            temp_angle = angles.firstAngle;
        }
        telemetry.addData("exit (ST) while, angle: ", angles.firstAngle);

        //reset motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
        if (turncw) {
            motorRight.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
        }
        telemetry.addData("Stopped motors", " Done");
        telemetry.update();

        sleep(100);

        // Use DogeCV to get sampling order
        telemetry.addData("Step3", "Use DogeCV to get sampling order");
        telemetry.update();

        detector.enable();
        //sleep(250);

        // reset the timeout time before starting
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 0.5) && !detector.isFound()) {
            Thread.yield();
        }

        //default all position variables
        leftPos = false;
        centerPos = false;
        rightPos = false;
        foundelement = false;

        //sleep(100);

        // scan for minerals and turn to left if not found
        if (detector.isFound()) {

            if ((detector.getXPosition() >= 1) && (detector.getXPosition() <= 213.5)) {
                rightPos = true;
                foundelement = true;
                telemetry.addData("Step3b", "Use DogeCV locate - found it -> RIGHT");

            } else if ((detector.getXPosition() >= 213.6) && (detector.getXPosition() <= 426.5)) {
                centerPos = true;
                foundelement = true;
                telemetry.addData("Step3b", "Use DogeCV locate - found it -> CENTER");

            } else if ((detector.getXPosition() >= 426.6) && (detector.getXPosition() <= 640)) {
                leftPos = true;
                foundelement = true;
                telemetry.addData("Step3b", "Use DogeCV locate - found it -> LEFT");

            }
        } else {

            //Try Rotate 30 deg left
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            temp_angle = angles.firstAngle;
            motorRight.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("Rotate 30 deg left - angle: ", angles.firstAngle);
            while (abs(temp_angle) > 150) {  //(temp_angle < -165) && (temp_angle > 160)
                motorLeft.setPower(turnPower);
                motorRight.setPower(turnPower);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                temp_angle = angles.firstAngle;
            }
            telemetry.addData("exit while (left), angle: ", angles.firstAngle);
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorRight.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("Stopped motors", " Done");
            telemetry.update();

            sleep(100);

            if (detector.isFound()) {
                if ((detector.getXPosition() >= 1) && (detector.getXPosition() <= 200)) {
                    centerPos = true;
                    foundelement = true;
                    telemetry.addData("Step3b", "Use DogeCV locate - found it -> CENTER");

                } else if ((detector.getXPosition() > 200) && (detector.getXPosition() <= 640)) {
                    leftPos = true;
                    foundelement = true;
                    telemetry.addData("Step3b", "Use DogeCV locate - found it -> LEFT");

                }
            } else {
                //Try Rotate 30 deg right
                temp_angle = angles.firstAngle;
                motorRight.setDirection(DcMotor.Direction.REVERSE);
                motorLeft.setDirection(DcMotor.Direction.REVERSE);
                telemetry.addData("Rotate 30 deg right - angle: ", angles.firstAngle);
                while ((temp_angle > 0) || (temp_angle < -158)) {  //(abs(temp_angle) > 158) {
                    motorLeft.setPower(turnPower);
                    motorRight.setPower(turnPower);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    temp_angle = angles.firstAngle;
                }
                telemetry.addData("exit (right) while, angle: ", angles.firstAngle);
                motorLeft.setPower(0);
                motorRight.setPower(0);
                motorLeft.setDirection(DcMotor.Direction.FORWARD);
                telemetry.addData("Stopped motors", " Done");
                telemetry.update();


                targetDrDistInch = 1.5f;
                eDrive(targetPower, targetDrDistInch, targetDrDistInch, 1);

                sleep(100);

                if (detector.isFound()) {
                    if ((detector.getXPosition() >= 1) && (detector.getXPosition() <= 440)) {
                        rightPos = true;
                        foundelement = true;
                        telemetry.addData("Step3b", "Use DogeCV locate - found it -> RIGHT");

                    } else if ((detector.getXPosition() > 440) && (detector.getXPosition() <= 426.5)) {
                        centerPos = true;
                        foundelement = true;
                        telemetry.addData("Step3b", "Use DogeCV locate - found it -> CENTER");

                    }
                } else {
                    telemetry.addData("Step3b", "Use DogeCV locate - couldn't find it, going with default");
                    centerPos = true;
                }
            }
        }


        telemetry.update();

        // Turn off DogeCV
        detector.disable();
        //CameraDevice.getInstance().setFlashTorchMode(false) ; //turns flash off

        // sample gold element
        telemetry.addData("Step4", "Select gold element");
        telemetry.update();

        targetDrDistInch = -18f; //default to center
        targetPower = DEFAULT_MOVE_SPEED;  // Set power

        //turn to mineral depending on position
        if (leftPos) {
            tangle = 135;
            motorRight.setDirection(DcMotor.Direction.FORWARD);
            while ((temp_angle > 135) || (temp_angle < 0)) {
                tempratio = (float) (abs(temp_angle - tangle) / 180);
                tempspeed = (float) turnPower * tempratio + 0.3f;
                motorLeft.setPower(tempspeed);
                motorRight.setPower(tempspeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                temp_angle = angles.firstAngle;
            }
            telemetry.addData("exit while, angle: ", angles.firstAngle);
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorRight.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("Stopped motors", " Done");
            telemetry.update();

            //eturn(turnPower,-28.0,true,5);

            //targetDrDistInch = -28f; // Set target distance - left element
            telemetry.addData("Step4b", "Gold element at left position");
            telemetry.update();
            //eDrive(targetPower,-1.0,0,500);
        } else if (centerPos) {
            tangle = 175;
            if ((temp_angle > 0) && (temp_angle < tangle) ) {
                motorLeft.setDirection(DcMotor.Direction.REVERSE);
                while (temp_angle < 175) {
                    tempratio = (float) (abs(temp_angle - tangle) / 180);
                    tempspeed = (float) turnPower * tempratio + 0.3f;
                    motorLeft.setPower(tempspeed);
                    motorRight.setPower(tempspeed);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    temp_angle = angles.firstAngle;
                }
                    telemetry.addData("exit while, angle: ", angles.firstAngle);
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setDirection(DcMotor.Direction.FORWARD);

            } else {
                motorRight.setDirection(DcMotor.Direction.FORWARD);
                while ((temp_angle > 175) || (temp_angle < 0)) {
                    tempratio = (float) (abs(abs(temp_angle) - tangle) / 180);
                    tempspeed = (float) turnPower * tempratio + 0.3f;
                    motorLeft.setPower(tempspeed);
                    motorRight.setPower(tempspeed);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    temp_angle = angles.firstAngle;
                }
                    telemetry.addData("exit while, angle: ", angles.firstAngle);
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorRight.setDirection(DcMotor.Direction.REVERSE);

            }
            telemetry.addData("Stopped motors", " Done");
            telemetry.update();

            //targetDrDistInch = -26f; // Set target distance - center element
            telemetry.addData("Step4b", "Gold element at center position");
            telemetry.update();
        } else if (rightPos) {
            tangle = -150;
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            while ((temp_angle < -150) || (temp_angle > 0)) {
                tempratio = (float) (abs(temp_angle - tangle) / 180);
                tempspeed = (float) turnPower * tempratio + 0.3f;
                motorLeft.setPower(tempspeed);
                motorRight.setPower(tempspeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                temp_angle = angles.firstAngle;
                //telemetry.addData("current angle: ", temp_angle);
                //telemetry.update();
            }
            telemetry.addData("exit while, angle: ", angles.firstAngle);
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("Stopped motors", " Done");
            telemetry.update();

            //eturn(turnPower,23,false,5);

            //targetDrDistInch = -28f; // Set target distance - right element
            telemetry.addData("Step4b", "Gold element at right position");
            telemetry.update();
            //eDrive(targetPower,0,-1.0,500);
        } else {
            telemetry.addData("Step4b", " - no element info... going with default");
            telemetry.update();

            tangle = 175;
            if ((temp_angle > 0) && (temp_angle < tangle) ) {
                motorLeft.setDirection(DcMotor.Direction.REVERSE);
                while (temp_angle < 175) {
                    tempratio = (float) (abs(temp_angle - tangle) / 180);
                    tempspeed = (float) turnPower * tempratio + 0.3f;
                    motorLeft.setPower(tempspeed);
                    motorRight.setPower(tempspeed);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    temp_angle = angles.firstAngle;
                }
                    telemetry.addData("exit while, angle: ", angles.firstAngle);
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setDirection(DcMotor.Direction.FORWARD);

            } else {
                motorRight.setDirection(DcMotor.Direction.FORWARD);
                while ((temp_angle > 175) || (temp_angle < 0)) {
                    tempratio = (float) (abs(abs(temp_angle) - tangle) / 180);
                    tempspeed = (float) turnPower * tempratio + 0.3f;
                    motorLeft.setPower(tempspeed);
                    motorRight.setPower(tempspeed);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    temp_angle = angles.firstAngle;
                }
                    telemetry.addData("exit while, angle: ", angles.firstAngle);
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorRight.setDirection(DcMotor.Direction.REVERSE);
            }
            telemetry.addData("Stopped motors", " Done");
            telemetry.update();

            //targetDrDistInch = -26f; // Set target distance - center element
            telemetry.addData("Step4b", "Gold element at center position");
            telemetry.update();
        }

        eDrive(targetPower, targetDrDistInch, targetDrDistInch, 9);

        //move toward rover
        //determine turn direction
        if (temp_angle > 0) {
            motorRight.setDirection(DcMotor.Direction.REVERSE);
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            turncw = false;
        } else {
            motorRight.setDirection(DcMotor.Direction.FORWARD);
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
            turncw = true;
        }

        //turn to 180
        telemetry.addData("Rotate to 180 - angle: ", angles.firstAngle);
        while (abs(temp_angle) < 178) {
            motorLeft.setPower(turnPower);
            motorRight.setPower(turnPower);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            temp_angle = angles.firstAngle;
        }
        telemetry.addData("exit (ST) while, angle: ", angles.firstAngle);

        //reset motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
        if (turncw) {
            motorRight.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
        }
        telemetry.addData("Stopped motors", " Done");
        telemetry.update();

        sleep(100);

        targetDrDistInch = 12f;
        eDrive(targetPower,targetDrDistInch,targetDrDistInch,5);

        //turn to +90

        tangle = 90;
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        while ((temp_angle > 90) || (temp_angle < 0)) {
            tempratio = (float) (abs(temp_angle - tangle) / 180);
            tempspeed = (float) turnPower * tempratio + 0.3f;
            motorLeft.setPower(tempspeed);
            motorRight.setPower(tempspeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            temp_angle = angles.firstAngle;
        }
        telemetry.addData("exit while, angle: ", angles.firstAngle);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Stopped motors", " Done");
        telemetry.update();

        sleep(100);

        targetDrDistInch = 27f;
        if (centerPos) {
            targetDrDistInch +=15;
        } else if (leftPos) {
            targetDrDistInch +=30;
        }

        eDrive(targetPower,targetDrDistInch,targetDrDistInch,5);

        //head to depot
        tangle = 135;
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        while (temp_angle < 135) {
            tempratio = (float) (abs(temp_angle - tangle) / 180);
            tempspeed = (float) turnPower * tempratio + 0.3f;
            motorLeft.setPower(tempspeed);
            motorRight.setPower(tempspeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            temp_angle = angles.firstAngle;
        }
        telemetry.addData("exit while, angle: ", angles.firstAngle);
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Stopped motors", " Done");
        telemetry.update();

        sleep(100);

        targetDrDistInch = 44f;

        eDrive(targetPower,targetDrDistInch,targetDrDistInch,7);

        // Deposit marker
        telemetry.addData("Step7", "Deposit marker");    //
        telemetry.update();

        runtime.reset();
        motorArm.setPower(1);
        while (opModeIsActive() &&
                (runtime.seconds() < 0.35)) {
        }
        runtime.reset();
        //motorArm.setDirection(DcMotor.Direction.REVERSE);
        //motorArm.setPower(1);
        motorArm.setPower(-1);
        while (opModeIsActive() &&
                (runtime.seconds()  < 0.7)) {
        }
        //motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setPower(0);
        //Done
        //sleep(100);     // pause for motors to stop move

        //debug stuff
        //sleep(3000);

        //lower the hook
        //eLift(1, (-4350 / ENCODER_CNT_PER_IN_DRIVE), 10);
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
            newLeftTarget = motorLeft.getCurrentPosition() + (int) (leftInches * ENCODER_CNT_PER_IN_DRIVE);
            newRightTarget = motorRight.getCurrentPosition() + (int) (rightInches * ENCODER_CNT_PER_IN_DRIVE);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft.setPower(abs(speed));
            motorRight.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy())) {
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void eLift(double speed,
                      double Inches, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = motorLift.getCurrentPosition() + (int) (Inches * ENCODER_CNT_PER_IN_DRIVE);
            motorLift.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLift.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLift.isBusy())) {
            }

            // Stop all motion;
            motorLift.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}