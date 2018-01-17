package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "DMRelicRedFrontV5", group = "RiderModes")
@Disabled
public class DMRelicRedFrontV5 extends DMRelicAbstract{

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

        telemetry.addData("I am alive - ","init");
        telemetry.addData("Debug mode: ", debug);

        telemetry.update();


    }

    @Override
    public void loop()
    {

        super.loop();

        // START ROBOT SEQUENCE
        // Establish the robot sequence of operation with the Switch operation.
        // The active Case (i.e., sequence step) is established by the value in seqRobot.
        // After a Case executes, Break the switch to prevent executing subsequent cases unintentionally.

        //turn off auto clear for telemetry
        telemetry.clear();

        Telemetry.Item modeItem = telemetry.addData("1. I am alive - ","init");
        Telemetry.Item debugItem = telemetry.addData("2. Debug mode: ", debug);
        Telemetry.Item debugnoteItem = telemetry.addData("2b. ---> ", " ------ ");
        Telemetry.Item seqItem = telemetry.addData("3a. Sequence # ", seqRobot);
        Telemetry.Item caseItem = telemetry.addData("3b.    ---> ", "");
        Telemetry.Item casenoteItem = telemetry.addData("3c.     --> ", "");
        Telemetry.Item redItem = telemetry.addData("4a. Red value ", "");
        Telemetry.Item blueItem = telemetry.addData("4b. Blue value ", "");
        Telemetry.Item redblueItem = telemetry.addData("4c. --> Gem is: ", "");
        Telemetry.Item vuforiaItem = telemetry.addData("5a. Vuforia: ", "");
        Telemetry.Item vuforiamarkItem = telemetry.addData("5b. -->  ", "");
        Telemetry.Item targetdistItem = telemetry.addData("6a. Distance: ", "");
        Telemetry.Item targetpowerItem = telemetry.addData("6b. Power: ", "");
        telemetry.addData("Runtime", getRuntime());

        modeItem.setValue("running");
        debugItem.setValue(debug);
        seqItem.setValue(seqRobot);
        caseItem.setValue("Entering switch");
        telemetry.update();

        switch (seqRobot) {

            case 1:
            case 10:
            case 18:
            case 22:
            case 26:
            case 30:
            case 36:
            case 42:
            case 46:
                {  //Reset encoder
                    resetME(0);  //function to reset encoders to 0

                    //Update telemetry data
                    seqItem.setValue(seqRobot);
                    caseItem.setValue("Reset encoders");

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME/4);
                }

                //if (seqRobot == 1) {
                //    seqRobot = 4;
                //} else {
                    seqRobot += 2;
                //}

                break;
            }

            case 3: {  //initialize the robot and load glyph
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Init robot and grab glyph");
                telemetry.update();

                //Gem arm up
                sGem.setPosition(0);
                //Grab glyph
                sGlyphL.setPosition(0.2);
                sGlyphR.setPosition(0.8);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }

                seqRobot ++;
                break;
            }

            case 4: {  //Lower gem arm
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Lower gem arm");
                telemetry.update();

                sGem.setPosition(0.8);  //lowers gem arm to 80% of 180 degree movement

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 6: {  // check color and push gem

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Check color and push gem");
                redItem.setValue(snColor.red());
                blueItem.setValue(snColor.blue());
                telemetry.update();

                sleep(SLEEP_TIME); // Pause to read color
                if (snColor.red() > snColor.blue()) {  //move forward to push red gem
                    redblueItem.setValue("RED");
                    telemetry.update();
                    targetDrDistInch = -GEM_DISTANCE; // Set target distance - forward
                }
                else {  //move back to push red gem
                    redblueItem.setValue("BLUE");
                    telemetry.update();
                    targetDrDistInch = GEM_DISTANCE; // Set target distance - back
                }

                targetDrRotateDeg = 0f;  // Not used for this
                targetPower = DEFAULT_MOVE_SPEED/2;  // Set power

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                // Use this OpModes's custom cmdMoveA method to calculate new target (in encoder
                // counts) and to initiate the move. cmdMoveA initiates a relative move.
                // cmdMove Parameters (distance inches, encoder count per inch, power, motor).
                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 8: {  // Lift jem arm, move robot back to starting position and turn off color sensor
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move robot back to starting position and lift gem arm");
                telemetry.update();

                sGem.setPosition(0); //move gem arm to start position
                snColor.enableLed(false); //turn off color sensor LED - does not seem to work

                movebackME(0,DEFAULT_MOVE_SPEED/2);  //use movebacME function to move motors back to provided position using given power (position,power)

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }

                seqRobot+=2;
                break;
            }

            case 12: { // Rotate to face decryption key

                //not needed....

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Not in use...");
                telemetry.update();

                seqRobot+=2;
                break;
            }

            case 14:  // Use vuforia to scan decryption key
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Use vuforia to read cypher");
                telemetry.update();

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
                //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
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

                sleep(SLEEP_TIME/2); //pause after reading

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                {
                        vuforiaItem.setValue("VuMark", "%s visible", vuMark);

                        if (vuMark == RelicRecoveryVuMark.LEFT)
                        {
                            leftCol = true;
                            centerCol = false;
                            rightCol = false;
                            vuforiamarkItem.setValue("LEFT");
                        }
                        else if (vuMark == RelicRecoveryVuMark.CENTER )
                        {
                            leftCol = false;
                            centerCol = true;
                            rightCol = false;
                            vuforiamarkItem.setValue("CENTER");
                        }
                        else if (vuMark == RelicRecoveryVuMark.RIGHT )
                        {
                            leftCol = false;
                            centerCol = false;
                            rightCol = true;
                            vuforiamarkItem.setValue("RIGHT");
                        }

                    }
                    else
                    {
                        vuforiaItem.setValue("VuMark not visible");
                        leftCol = false;
                        centerCol = true;
                        rightCol = false;
                        vuforiamarkItem.setValue("Defaulting to center column");
                    }

                    //telemetry.update();

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=4;
                break;
            }



            case 20:  // Move robot to correct column
                        //27", 36", 43" - 9"
            {

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move robot to correct column");
                telemetry.update();

                targetDrRotateDeg = 0f; //not used
                targetPower = DEFAULT_MOVE_SPEED;  // Set power
                targetDrDistInch = -35.6f; //default to center

                if (leftCol)
                {
                    targetDrDistInch = -43.3f; // Set target distance - left column

                } else if (centerCol)
                {
                    targetDrDistInch = -35.6f; // Set target distance - center column

                } else if (rightCol)
                {
                    targetDrDistInch = -27.5f; // Set target distance - right column

                } else {
                    casenoteItem.setValue(" - no column info... going with default");
                }

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME*4);  //Sleep for 4x the normal sleep length
                }

                //seqRobot++;
                seqRobot +=2;
                casenoteItem.setValue("");
                break;
            }


            case 24:  // Rotate Left - to place Glyph
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Rotate Left - to place Glyph");
                telemetry.update();

                motorLeftA.setTargetPosition(-GLYPH_ROTATE);
                motorLeftB.setTargetPosition(-GLYPH_ROTATE);
                motorRightA.setTargetPosition(GLYPH_ROTATE);
                motorRightB.setTargetPosition(GLYPH_ROTATE);

                targetPower = DEFAULT_MOVE_SPEED;

                targetdistItem.setValue("encoders = " + GLYPH_ROTATE);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                motorLeftA.setPower(targetPower);
                motorLeftB.setPower(targetPower);
                motorRightA.setPower(targetPower);
                motorRightB.setPower(targetPower);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME*2);
                }

                //seqRobot++;
                seqRobot +=2;
                break;
            }


            case 28: // move forward 7 "
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move forward 7 \"");
                telemetry.update();

                targetDrRotateDeg = 0f;
                targetDrDistInch = 7f; // Set target distance
                targetPower = DEFAULT_MOVE_SPEED;  // Set power

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME*2);
                }

                //seqRobot++;
                seqRobot +=2;
                break;
            }


            case 32:  // open Glyph
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Open Glyph arm");
                telemetry.update();

                sGlyphL.setPosition(0.09);
                sGlyphR.setPosition(0.9);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 34: {  // Move robot back 6"
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move robot back 6 \"");
                telemetry.update();

                targetDrRotateDeg = 0f;
                targetDrDistInch = -6f; // Set target distance
                targetPower = DEFAULT_MOVE_SPEED;  // Set power

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot +=2;
                break;
            }

            case 38:  // Close glyph
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Close Glyph arm");
                telemetry.update();

                sGlyphL.setPosition(0.4);
                sGlyphR.setPosition(0.5);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot+=2;
                break;
            }

            case 40:  // move forward 6.5"
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move froward 6.5\"");
                telemetry.update();

                targetDrRotateDeg = 0f;
                targetDrDistInch = 6.5f; // Set target distance
                targetPower = DEFAULT_MOVE_SPEED;  // Set power

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                seqRobot +=2;
                break;
            }

            case 44: // move back 2.5 "
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move back 2.5\"");
                telemetry.update();

                targetDrRotateDeg = 0f;
                targetDrDistInch = -2.5f; // Set target distance
                targetPower = DEFAULT_MOVE_SPEED;  // Set power

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeftA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveA(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME);
                }

                //seqRobot++;
                seqRobot +=2;
                break;
            }


            case 99:  // Done
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Case 99 - DONE");

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(400);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }
                break;
            } //end case 99

            default:
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Default case....");

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                }  else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME/2);
                }
                break;
            } //end default




        } //end switch


        telemetry.update();


        } // End OpMode Loop Method






    }