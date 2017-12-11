package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static android.os.SystemClock.sleep;

/* ------------------------------------------------------------------
 * This Op Mode is a template for Autonomous Control
 *
 * Gamepads
 * 	Not used; however, for troubleshooting prior to competition, Gamepad1's "X" button will allow
 * 	the programmer to enter debug mode (in init).
 * ------------------------------------------------------------------
 */

@Autonomous(name = "DMRelicBlueFrontV4", group = "RiderModes")
public class DMRelicBlueFrontV4 extends DMRelicAbstract{

    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override
    public void init() {

        super.init();

        //Set debug to false
        debug = false;

        if(gamepad1.x) {
            debug = true;
        }

        //set motors to use encoders
        motorLeftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //turn off auto clear for telemetry
        telemetry.setAutoClear(false);

    }

    @Override
    public void loop()
    {

        super.loop();

        // START ROBOT SEQUENCE
        // Establish the robot sequence of operation with the Switch operation.
        // The active Case (i.e., sequence step) is established by the value in seqRobot.
        // After a Case executes, Break the switch to prevent executing subsequent cases unintentionally.

        telemetry.addData("Entering switch - seq = ", seqRobot);
        telemetry.addData("Debug - ", debug);
        telemetry.update();

        switch (seqRobot) {

            case 1:
            case 10:
            case 18:
            case 22:
            case 26:
            case 30:
            case 36:
            case 44:
                {  //Reset encoder
                resetME(0);  //function to reset encoders to 0

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Reset encoders", "");
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Reset encoders", "");
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }

                //if (seqRobot == 1) {
                //    seqRobot = 4;
                //} else {
                    seqRobot += 2;
                //}

                break;
            }

            case 3: {  //initialize the robot and load glyph
                //Gem arm up
                sGem.setPosition(0);
                //Grab glyph
                sGlyphL.setPosition(0.2);
                sGlyphR.setPosition(0.8);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Init robot and load glyph", "");
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Init robot and load glyph", "");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot ++;
                break;
            }

            case 4: {  //Lower gem arm
                sGem.setPosition(0.8);  //lowers gem arm to 80% of 180 degree movement

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Lower gem arm", "");
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Lower gem arm", "");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 6: {  // check color and push gem

                telemetry.addData("Alpha", snColor.alpha());
                telemetry.addData("Red  ", snColor.red());
                telemetry.addData("Green", snColor.green());
                telemetry.addData("Blue ", snColor.blue());
                telemetry.addData("Entering color check", " -- IF --");
                sleep(SLEEP_TIME); // Pause to read color
                if (snColor.red() > snColor.blue()) {  //move forward to push red gem
                    telemetry.addData("-----", " Gem Color");
                    telemetry.addData("Gem ", "RED");
                    telemetry.addData("-----", " Gem Color");
                    targetDrDistInch = 3f; // Set target distance - forward
                }
                else {  //move back to push red gem
                    telemetry.addData("-----", " Gem Color");
                    telemetry.addData("Gem ", "BLUE");
                    telemetry.addData("-----", " Gem Color");
                    targetDrDistInch = -3f; // Set target distance - back
                }
                telemetry.addData("Exit color check", " -- IF --");

                if (debug) {
                    telemetry.update();
                    sleep(300);
                }
                targetDrRotateDeg = 0f;  // Not used for this
                targetPower = 0.3f;  // Set power

                // Use this OpModes's custom cmdMoveA method to calculate new target (in encoder
                // counts) and to initiate the move. cmdMoveA initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("Alpha", snColor.alpha());
                        telemetry.addData("Red  ", snColor.red());
                        telemetry.addData("Green", snColor.green());
                        telemetry.addData("Blue ", snColor.blue());
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("Alpha", snColor.alpha());
                    telemetry.addData("Red  ", snColor.red());
                    telemetry.addData("Green", snColor.green());
                    telemetry.addData("Blue ", snColor.blue());
                    telemetry.addData("In case ", seqRobot);
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 8: {  // Lift jem arm, move robot back to starting position and turn off color sensor
                sGem.setPosition(0); //move gem arm to start position
                snColor.enableLed(false); //turn off color sensor LED - does not seem to work

                movebackME(0,0.3f);  //use movebacME function to move motors back to provided position using given power (position,power)

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case - ", seqRobot);
                        telemetry.addData("Move robot back to starting position", " ");
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case - ", seqRobot);
                    telemetry.addData("Move robot back to starting position", " ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 12: { // Rotate to face decryption key

                //not needed....

/*
                motorLeftA.setTargetPosition(DECRIPT_ROTATE);
                motorLeftB.setTargetPosition(DECRIPT_ROTATE);
                motorRightA.setTargetPosition(-DECRIPT_ROTATE);
                motorRightB.setTargetPosition(-DECRIPT_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(400);
                }
*/
                seqRobot+=2;
                break;
            }

            case 14:  // Setup vuforia and scan decryption key
            {
                /*
                * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
                * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
                */
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                //Turned off the screen
                //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);  //screen is on
                // OR...  Do Not Activate the Camera Monitor View, to save power
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();  //screen is off

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

                sleep(SLEEP_TIME/2);  // Pause before reading
                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                relicTrackables.activate();

                sleep(SLEEP_TIME/2); //pause after reding

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

                    }
                    else
                    {
                        telemetry.addData("VuMark", "not visible");
                        leftCol = false;
                        centerCol = true;
                        rightCol = false;
                        telemetry.addData("Defaulting to ", "center column");
                    }


                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("vuforia", "");
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("vuforia", "");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=4;  //skip 16 and go to 18
                break;
            }

/*
            case 16: // Reset position after vuforia
            {
                movebackME(0,0.2f);  //use movebacME function to move motors back to provided position using given power (position,power)

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                }

                seqRobot+=2;
                break;
            }
*/

            case 20:  // Move robot to correct column
                        //27", 34", 41" - 9"
            {

                targetDrRotateDeg = 0f; //not used
                targetPower = 0.4f;  // Set power
                targetDrDistInch = 36f; //default to center

                if (leftCol)
                {
                    targetDrDistInch = 28f; // Set target distance - left column

                } else if (centerCol)
                {
                    targetDrDistInch = 36f; // Set target distance - center column

                } else if (rightCol)
                {
                    targetDrDistInch = 43f; // Set target distance - right column

                }

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Move robot in front of a column", "");
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Move robot in front of a column", "");
                    telemetry.addData("Target Distance: ", targetDrDistInch);
                    telemetry.addData("Target Power: ", targetPower);
                    telemetry.update();
                    sleep(SLEEP_TIME*3);  //Sleep for 3x the normal sleep length
                }

                seqRobot++;
                break;
            }

            case 21:
            case 29:
            case 35:    // Hold until drive train move is complete
            {
                // Use this OpModes's custom chkMove to determine if motor move(s) are complete
                // chkMove Parameters (motor, target, allowed +/- error counts from target)
                // May need to add motor-is-busy check to ensure electric breaking complete.
                // May need to compensate for motor power if one motor is faster than another to keep straight line.
                if (chkMove(motorLeftA, targetPosLeftA, ERROR_DRV_POS) &&
                        chkMove(motorRightA, targetPosRightA, ERROR_DRV_POS) &&
                            chkMove(motorLeftB, targetPosLeftB, ERROR_DRV_POS) &&
                                chkMove(motorRightB, targetPosRightB, ERROR_DRV_POS))
                {    // If drive train at target, hold position for 0.1s to stabilize motors.
                    sleep(100);

                    if (debug) {
                        while (!gamepad1.b) {
                            telemetry.addData("In case ", seqRobot);
                            telemetry.addData("Please press B to continue", "");
                            telemetry.update();
                        }
                        sleep(200);
                    } else {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.update();
                        sleep(SLEEP_TIME/2);
                    }

                    seqRobot++;
                    break;
                }

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Waiting for motors to stop", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else  {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Waiting for motors to stop", "");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

            }

            case 24:  // Rotate Left - to place Glyph
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
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Please press B to continue", "");
                    telemetry.update();
                    sleep(SLEEP_TIME*2);
                }

                seqRobot++;
                break;
            }

            case 25:
            case 39:    // Check to see if the turn was up to standards
            {
                telemetry.addData("Checking to see if turn was completed", "");
                if (motorLeftA.getCurrentPosition() <= -GLYPH_ROTATE+ERROR_DRV_POS && motorLeftA.getCurrentPosition() >= -GLYPH_ROTATE-ERROR_DRV_POS &&
                        motorLeftB.getCurrentPosition() <= -GLYPH_ROTATE+ERROR_DRV_POS && motorLeftB.getCurrentPosition() >= -GLYPH_ROTATE-ERROR_DRV_POS &&
                            motorRightA.getCurrentPosition() >= GLYPH_ROTATE-ERROR_DRV_POS && motorRightA.getCurrentPosition() <= GLYPH_ROTATE+ERROR_DRV_POS &&
                                motorRightB.getCurrentPosition() >= GLYPH_ROTATE-ERROR_DRV_POS && motorRightB.getCurrentPosition() <= GLYPH_ROTATE+ERROR_DRV_POS )
                {
                    if (debug) {
                        while (!gamepad1.b) {
                            telemetry.addData("In case ", seqRobot);
                            telemetry.addData("Turn successful - moving to next step", "");
                            telemetry.update();
                        }
                        sleep(200);
                    } else {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Turn successful - moving to next step", "");
                        telemetry.update();
                        sleep(SLEEP_TIME);
                    }

                    seqRobot++;
                    break;
                }
                else
                {
                    if (debug) {
                        while (!gamepad1.b) {
                            telemetry.addData("In case ", seqRobot);
                            telemetry.addData("Turn unsuccessful - going back to turn", "");
                            telemetry.update();
                        }
                        sleep(200);
                    } else {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Turn unsuccessful - going back to turn", "");
                        telemetry.update();
                        sleep(SLEEP_TIME);
                    }

                    seqRobot--;
                    break;
                }
            }

            case 28: // move forward 7 "
            {

                targetDrRotateDeg = 0f;
                targetDrDistInch = 7f; // Set target distance
                targetPower = 0.2f;  // Set power

                targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Please press B to continue", "");
                    telemetry.update();
                    sleep(SLEEP_TIME*2);
                }

                seqRobot++;
                break;
            }


            case 32:  // open Glyph
            {
                sGlyphL.setPosition(0.09);
                sGlyphR.setPosition(0.9);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.addData("Please press B to continue", "");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 34:  // move back
            {
                targetDrRotateDeg = 0f;
                targetDrDistInch = -7f; // Set target distance
                targetPower = 0.2f;  // Set power

                targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.update();
                    sleep(SLEEP_TIME*2);
                }

                seqRobot++;
                break;
            }


            case 38:  // rotate 180 deg
            {
                motorLeftA.setTargetPosition(END_ROTATE);
                motorLeftB.setTargetPosition(END_ROTATE);
                motorRightA.setTargetPosition(-END_ROTATE);
                motorRightB.setTargetPosition(-END_ROTATE);
                motorLeftA.setPower(.3);
                motorLeftB.setPower(.3);
                motorRightA.setPower(.3);
                motorRightB.setPower(.3);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.update();
                    sleep(SLEEP_TIME*3);
                }

                seqRobot++;
                break;
            }

            case 42: // move back 5 "
            {

                targetDrRotateDeg = 0f;
                targetDrDistInch = -5f; // Set target distance
                targetPower = 0.3f;  // Set power

                targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot++;
                break;
            }


            case 99:  // Done
            {
                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(400);
                } else {
                    telemetry.addData("In case ", seqRobot);
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }
                break;
            } //end case 99

            default:
            {
                if (debug) {
                    while (!gamepad1.b) {
                        telemetry.addData("In case default - # ", seqRobot);
                        telemetry.addData("Please press B to continue", "");
                        telemetry.update();
                    }
                    sleep(400);
                }  else {
                    telemetry.addData("In case default - # ", seqRobot);
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }
                break;
            } //end default




        } //end switch


        telemetry.update();


        } // End OpMode Loop Method






    }