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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static android.os.SystemClock.sleep;

@TeleOp(name = "DMRelicBFTest")
public class DMRelicBFTest extends DMRelicAbstract {
    public DMRelicBFTest() {
    }

    VuforiaLocalizer vuforia;
    boolean IsRed = false;

    @Override
    public void init() {

        super.init();

        debug = true;

        // Set all motors to run with encoders
        motorRightA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IncVal = 5;
        Gdown = false;
        Gopen = true;

        //init glyph position
        sGlyphL.setPosition(0.1);
        sGlyphR.setPosition(0.8);

        //init gem arm position
        sGem.setPosition(0);

    }

    @Override
    public void loop() {

        super.loop();

        telemetry.addData("Entering switch - seq = ", seqRobot);
        telemetry.addData("Debug - ", debug);

        if (gamepad1.x) {
           if (debug) {
               debug = false;
           } else {
               debug = true;
           }
            telemetry.addData("Debug is now: ", debug);
            telemetry.update();
            sleep(500);
        }

        //for (int tempcounter = 1; tempcounter < 500; tempcounter++) {
            if (gamepad1.b) {
                seqRobot += 1;
                telemetry.addData("Moving to next step - seq = ", seqRobot);
                telemetry.update();

            } else {
                telemetry.addData("Please press B to continue ", seqRobot);
                telemetry.update();

            }
        //}
        sleep(500);

        switch (seqRobot) {

            case 1: {  //Reset encoder
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Reset encoder: ", seqRobot);
                        telemetry.addData("Reset encoders", "");
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot = 4;
                break;
            }

            case 4: {  //Lower gem arm
                sGem.setPosition(0.8);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Lower gem arm", "");
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 6: {  // check color and push gem
                //lightSensor.getLightDetected();

                //telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", snDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", snColor.alpha());
                telemetry.addData("Red  ", snColor.red());
                telemetry.addData("Green", snColor.green());
                telemetry.addData("Blue ", snColor.blue());
                telemetry.addData("Entering color check", " -- IF --");
                if (snColor.red() > snColor.blue()) {  //move forward
                    telemetry.addData("-----", " Gem Color");
                    telemetry.addData("Gem ", "RED");
                    telemetry.addData("-----", " Gem Color");
                    IsRed = true;
                    //targetPower = 0.2d;  // Set power
                    targetDrDistInch = 2.5f; // Set target distance
                }
                else {  //move back
                    telemetry.addData("-----", " Gem Color");
                    telemetry.addData("Gem ", "BLUE");
                    telemetry.addData("-----", " Gem Color");
                    IsRed = false;
                    //targetPower = (-0.2);  // Set power
                    targetDrDistInch = -2.5f; // Set target distance
                }
                telemetry.addData("Exit color check", " -- IF --");

                if (debug) {
                    telemetry.update();
                    sleep(500);
                }
                targetDrRotateDeg = 0f;
                targetPower = 0.2d;  // Set power

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                /*
                //Move back to start position
                targetPower *= -1;  // Set power

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                */

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("Alpha", snColor.alpha());
                        telemetry.addData("Red  ", snColor.red());
                        telemetry.addData("Green", snColor.green());
                        telemetry.addData("Blue ", snColor.blue());
                        telemetry.addData("In case - check color and push gem: ", seqRobot);
                    if (IsRed) {
                        telemetry.addData("Identified as ", "Red");
                    } else {
                        telemetry.addData("Identified as ", "Blue");
                    }
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 8: {  // Move robot back to starting position and reset gem arm
                sGem.setPosition(0);
                snColor.enableLed(false);

                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                motorLeftA.setPower(.2);
                motorLeftB.setPower(.2);
                motorRightA.setPower(.2);
                motorRightB.setPower(.2);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Move robot back to starting position: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 10: {  // Reset motors/encoders
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Reset motors/encoders: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 12: { // Rotate to face decryption key
                motorLeftA.setTargetPosition(DECRIPT_ROTATE);
                motorLeftB.setTargetPosition(DECRIPT_ROTATE);
                motorRightA.setTargetPosition(-DECRIPT_ROTATE);
                motorRightB.setTargetPosition(-DECRIPT_ROTATE);
                motorLeftA.setPower(.2);
                motorLeftB.setPower(.2);
                motorRightA.setPower(.2);
                motorRightB.setPower(.2);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Rotate to face decryption key: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 14:  // Setup vuforia and scan decryption key
            {
                /*
                * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
                * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
                */
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                // OR...  Do Not Activate the Camera Monitor View, to save power
                // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                parameters.vuforiaLicenseKey = "AaRImwv/////AAAAGUjFLy92WE8UlmWKUSNcUNYpTptTmSX6QFQYt5PR6Far3tcNpmDCqgAxQDXowjTxfxTraIFnoWUUuHv5BfINsEyM88rx4xY6G8yuq/ys88jHy+m7sKtzKHYVlMSpjQaPUx47BOFSaC97HlAXFBZ0a/gs4IE7q6TQFxUJxhsl4UEaJp0T79um2REaDI9N1Zd33XrUJMfM52gWFgLqny4pAQcXEAaORowMlFYMx2GjlhWFTlo17bCwGnLr8cEHwFpthWYHgIL8Be/bsNR4kdHz6KajNuNzP7U0dj0xG/yxJSlzuRzxjgKwxlHHMl2yWXks6kON/AUVyQRF6bxDGWkW/Y+WsO0XlQlpypXeRbjscV/H";

                /*
                * We also indicate which camera on the RC that we wish to use.
                * Here we chose the back (HiRes) camera (for greater range), but
                * for a competition robot, the front camera might be more convenient.
                */
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
                /**
                 * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
                 * in this data set: all three of the VuMarks in the game were created from this one template,
                 * but differ in their instance id information.
                 * @see VuMarkInstanceId
                 */
                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                relicTrackables.activate();

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {


                    telemetry.addData("VuMark", "%s visible", vuMark);

                    if (vuMark == RelicRecoveryVuMark.LEFT)
                    {
                        leftCol = true;
                        centerCol = false;
                        rightCol = false;
                    }
                    else if (vuMark == RelicRecoveryVuMark.CENTER )
                    {
                        leftCol = false;
                        centerCol = true;
                        rightCol = false;
                    }
                    else if (vuMark == RelicRecoveryVuMark.RIGHT )
                    {
                        leftCol = false;
                        centerCol = false;
                        rightCol = true;
                    }

                        /*
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

                        if (pose != null)
                        {
                            VectorF trans = pose.getTranslation();
                            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                            // Extract the X, Y, and Z components of the offset of the target relative to the robot
                            double tX = trans.get(0);
                            double tY = trans.get(1);
                            double tZ = trans.get(2);

                            // Extract the rotational components of the target relative to the robot
                            double rX = rot.firstAngle;
                            double rY = rot.secondAngle;
                            double rZ = rot.thirdAngle;
                        }
                        */
                }
                else
                {
                    telemetry.addData("VuMark", "not visible");
                }


                telemetry.update();

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Setup vuforia and scan decryption key: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 16: // Reset position after vuforia
            {
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                motorLeftA.setPower(.2);
                motorLeftB.setPower(.2);
                motorRightA.setPower(.2);
                motorRightB.setPower(.2);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Reset position after vuforia: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 18:  // Reset positions
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Reset positions: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }
            case 20:  // Move robot to correct column
                //28", 35", 42" - 9"
            {

                if (leftCol)
                {
                    targetDrRotateDeg = 0f;
                    targetDrDistInch = 19f; // Set target distance
                    targetPower = 0.2d;  // Set power

                    targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                    targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                    targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                    targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                } else if (centerCol)
                {
                    targetDrRotateDeg = 0f;
                    targetDrDistInch = 26f; // Set target distance
                    targetPower = 0.2d;  // Set power

                    targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                    targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                    targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                    targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                } else if (rightCol)
                {
                    targetDrRotateDeg = 0f;
                    targetDrDistInch = 33f; // Set target distance
                    targetPower = 0.2d;  // Set power

                    targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                    targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                    targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                    targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                }

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Move robot to correct column: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 22:  // Reset position
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Reset position: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    // telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 24:  // Rotate to Glyph
            {
                motorLeftA.setTargetPosition(-GLYPH_ROTATE);
                motorLeftB.setTargetPosition(-GLYPH_ROTATE);
                motorRightA.setTargetPosition(GLYPH_ROTATE);
                motorRightB.setTargetPosition(GLYPH_ROTATE);
                motorLeftA.setPower(.2);
                motorLeftB.setPower(.2);
                motorRightA.setPower(.2);
                motorRightB.setPower(.2);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - Rotate to Glyph: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 26:
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 28: // move forward 5 "
            {

                targetDrRotateDeg = 0f;
                targetDrDistInch = 5f; // Set target distance
                targetPower = 0.2d;  // Set power

                targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - move forward 5: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 30:
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 32:  // move and open Glyph
            {
                sGlyphL.setPosition(0.05);
                sGlyphR.setPosition(0.83);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - move and open Glyph: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 34:  // move back
            {
                targetDrRotateDeg = 0f;
                targetDrDistInch = -5f; // Set target distance
                targetPower = 0.2d;  // Set power

                targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case - move back: ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 36:
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 38:
            {
                motorLeftA.setTargetPosition(END_ROTATE);
                motorLeftB.setTargetPosition(END_ROTATE);
                motorRightA.setTargetPosition(-END_ROTATE);
                motorRightB.setTargetPosition(-END_ROTATE);
                motorLeftA.setPower(.2);
                motorLeftB.setPower(.2);
                motorRightA.setPower(.2);
                motorRightB.setPower(.2);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 40:
            {
                motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);

                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    sleep(400);
                }

                seqRobot+=1;
                break;
            }

            case 99:  // Done
            {
                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    //sleep(400);
                }
                break;
            }

            default:
            {
                if (debug) {
                    //while (!gamepad1.b) {
                        telemetry.addData("In case default - # ", seqRobot);
                    //    telemetry.addData("Please press B to continue", "");
                    //    telemetry.update();
                    //}
                    //sleep(400);
                }
                break;
            }




        }


        telemetry.update();
        sleep(300);
// End OpMode Loop Method
    }
    @Override
    public void stop ()
    {
        super.stop();
    }
}
