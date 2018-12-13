package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/* ------------------------------------------------------------------
 * This Op Mode is a template for Autonomous Control
 *
 * Gamepads
 * 	Not used; however, for troubleshooting prior to competition, Gamepad1's "X" button will allow
 * 	the programmer to enter debug mode (in init).
 * ------------------------------------------------------------------
 */

@Autonomous(name = "DM Rokus Depot v2", group = "Auto")
@Disabled
public class DMRokusDepotV2 extends DMRokus_Abstract{

    //------------------------------------------------------------------
    // Robot OpMode Loop Method
    //------------------------------------------------------------------

    SamplingOrderDetector detector;

    @Override
    public void init() {

        super.init();

        //Set debug to false
        debug = false;

        if(gamepad1.x) {
            debug = true;
        }

        //setup for sampling detector
        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        //detector.enable();

        //set motors to use encoders
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        Telemetry.Item IsFoundItem = telemetry.addData("4a. Found sampling ", "");
        Telemetry.Item SamplingItem = telemetry.addData("4b. Sampling Order ", "");
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
            case 8:
            case 14:
            case 20:
            case 24:
            case 28:
            case 32:

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

                    seqRobot += 2;

                break;
            }

            case 3: {  //Land the robot
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Land the robot");
                telemetry.update();

                //Lower the robot
                motorLift.setPower(1.0);
                sleep(3300);  //pause for 3.3 sec
                motorLift.setPower(0);

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

                seqRobot ++;
                break;
            }

            case 4: {  //Land the robot - phase 2
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Land the robot - 2");
                telemetry.update();

                //Lower the robot
                motorLift.setPower(1.0);
                sleep(3300);  //pause for 3.3 sec
                motorLift.setPower(0);

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

                seqRobot +=2;
                break;
            }

            case 6: {  //Land the robot phase 3
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Land the robot - 3");
                telemetry.update();

                //Lower the robot
                motorLift.setPower(1.0);
                sleep(3200);  //pause for 3.2 sec
                motorLift.setPower(0);

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

                seqRobot +=2;
                break;
            }


            case 10: {  //unlatch
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Unlatch");
                telemetry.update();

                motorLeft.setTargetPosition(250);
                motorRight.setTargetPosition(-250);

                targetPower = DEFAULT_MOVE_SPEED;

                targetdistItem.setValue("encoders = 250");
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                motorLeft.setPower(targetPower);
                motorRight.setPower(targetPower);

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

            case 12: {  // rotate ~180 deg

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Rotate ~180");
                telemetry.update();

                motorLeft.setTargetPosition(END_ROTATE);
                motorRight.setTargetPosition(-END_ROTATE);

                targetPower = DEFAULT_MOVE_SPEED*1.3f;

                targetdistItem.setValue("encoders = " + END_ROTATE);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                motorLeft.setPower(targetPower);
                motorRight.setPower(targetPower);

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

            case 16:  // Use DogeCV to get sampling order
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Use DogeCV to get sampling order");
                telemetry.update();

                sleep(SLEEP_TIME/4);  // Pause before reading

                detector.enable();

                sleep(1500); //pause after reading

                if (detector.isFound())
                {
                        IsFoundItem.setValue("Found it");

                        if (detector.getCurrentOrder().toString() == "LEFT")
                        {
                            leftPos = true;
                            centerPos = false;
                            rightPos = false;
                            SamplingItem.setValue("LEFT");
                        }
                        else if (detector.getCurrentOrder().toString() == "Center" )
                        {
                            leftPos = false;
                            centerPos = true;
                            rightPos = false;
                            SamplingItem.setValue("CENTER");
                        }
                        else if (detector.getCurrentOrder().toString() == "RIGHT" )
                        {
                            leftPos = false;
                            centerPos = false;
                            rightPos = true;
                            SamplingItem.setValue("RIGHT");
                        }

                }
                else
                {
                    IsFoundItem.setValue(" cant find it");
                    leftPos = false;
                    centerPos = false;
                    rightPos = true;
                    SamplingItem.setValue("Defaulting to RIGHT");
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
                    sleep(SLEEP_TIME/2);
                }

                seqRobot +=2;
                break;
            }

            case 18:  // Turn off DogeCV
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Turn off DogeCV");
                telemetry.update();

                detector.disable();

                sleep(SLEEP_TIME/4); //pause to disable

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

                seqRobot +=2;
                break;
            }

            case 22:  // sample gold element
                //Center -> L=30", C=26", R=30"
            {

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Select gold element");
                telemetry.update();

                targetDrRotateDeg = 0f; //center
                targetPower = DEFAULT_MOVE_SPEED;  // Set power
                targetDrDistInch = -26f; //default to center

                if (leftPos)
                {
                    targetDrDistInch = -30f; // Set target distance - left element
                    targetPosLeft = cmdMoveA(-1.0f, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeft);
                    casenoteItem.setValue("Gold element at left position");

                } else if (centerPos)
                {
                    targetDrDistInch = -26f; // Set target distance - center element
                    casenoteItem.setValue("Gold element at center position");

                } else if (rightPos)
                {
                    targetDrDistInch = -30f; // Set target distance - right element
                    targetPosRight = cmdMoveA(-1.0f, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRight);;
                    casenoteItem.setValue("Gold element at right position");

                } else {
                    casenoteItem.setValue(" - no element info... going with default");
                }

                sleep(800);

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeft = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeft);
                targetPosRight = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRight);

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

                seqRobot +=2;
                casenoteItem.setValue("");
                break;
            }

            case 26:  // move to depot
                //~ 18"
            {

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move to depot");
                telemetry.update();

                targetDrRotateDeg = 0f; //center
                targetPower = DEFAULT_MOVE_SPEED;  // Set power
                targetDrDistInch = -18f; //default to center

                if (leftPos)
                {
                    targetPosRight = cmdMoveA(-1.0f, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRight);

                    casenoteItem.setValue("Gold element at left position");

                } else if (centerPos)
                {
                    casenoteItem.setValue("Gold element at center position");

                } else if (rightPos)
                {
                    targetPosLeft = cmdMoveA(-1.0f, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeft);
                    casenoteItem.setValue("Gold element at right position");

                } else {
                    casenoteItem.setValue(" - no element info... going with default");
                }

                sleep(800);

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                targetPosLeft = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeft);
                targetPosRight = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRight);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME*3);  //Sleep for 4x the normal sleep length
                }

                seqRobot +=2;
                casenoteItem.setValue("");
                break;
            }

            case 30: {  // rotate ~180 deg

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Rotate ~180");
                telemetry.update();

                motorLeft.setTargetPosition((END_ROTATE+50));
                motorRight.setTargetPosition(-(END_ROTATE+50));

                targetPower = DEFAULT_MOVE_SPEED*1.5f;

                targetdistItem.setValue("encoders = " + END_ROTATE);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                motorLeft.setPower(targetPower);
                motorRight.setPower(targetPower);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(SLEEP_TIME+200);
                }

                seqRobot+=2;
                break;
            }

            case 34:  //deposit marker

            {

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Deposit marker");
                telemetry.update();

                motorArm.setPower(1);

                sleep(1350);

                motorArm.setPower(-1);

                if (debug) {
                    while (!gamepad1.b) {
                        debugnoteItem.setValue("Please press B to continue");
                        telemetry.update();
                    }
                    sleep(200);
                } else {
                    debugnoteItem.setValue("  -----  ");
                    telemetry.update();
                    sleep(1800);
                }

                seqRobot =99; //+=2;
                casenoteItem.setValue("");
                break;
            }
            /*
            case 17:  // Move forward 23"
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move robot forward 23 \"");
                telemetry.update();

                targetDrRotateDeg = 0f; //not used
                targetPower = DEFAULT_MOVE_SPEED;  // Set power
                targetDrDistInch = 23f;

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

                seqRobot++;
                break;

            }

            case 20:  // Move robot to correct column
                        //Right -> L=5", C=12.5", R=20"
            {

                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move robot to correct column");
                telemetry.update();

                targetDrRotateDeg = 0f; //not used
                targetPower = DEFAULT_MOVE_SPEED;  // Set power
                targetDrDistInch = 20f; //default to center

                if (leftCol)
                {
                    targetDrDistInch = 8f; // Set target distance - left column - 5

                } else if (centerCol)
                {
                    targetDrDistInch = 20f; // Set target distance - center column - 12.5

                } else if (rightCol)
                {
                    targetDrDistInch = 32f; // Set target distance - right column - 20

                } else {
                    casenoteItem.setValue(" - no column info... going with default");
                }

                targetdistItem.setValue(targetDrDistInch);
                targetpowerItem.setValue(targetPower);
                telemetry.update();

                //For strafe we need to change power direction for each wheel

                targetPosLeftA = cmdMoveA(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveA(-targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveA(-targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
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

            case 24: // move forward 9 "
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move forward 9 \"");
                telemetry.update();

                targetDrRotateDeg = 0f;
                targetDrDistInch = 9f; // Set target distance
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
                seqRobot +=4;  //skip to step 30
                break;
            }


            case 30:  // open Glyph
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Open Glyph arm");
                telemetry.update();

                sGlyphL.setPosition(0.09);
                sGlyphR.setPosition(0.28);  //from .9

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

            case 32: {  // Move robot back 6"
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

            case 36:  // Close glyph
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Close Glyph arm");
                telemetry.update();

                sGlyphL.setPosition(0.4);
                sGlyphR.setPosition(0.14);  //from .5

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

            case 38:  // move forward 6.5"
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

        case 42: // move back 3 "
            {
                //Update telemetry data
                seqItem.setValue(seqRobot);
                caseItem.setValue("Move back 3\"");
                telemetry.update();

                targetDrRotateDeg = 0f;
                targetDrDistInch = -3f; // Set target distance
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

*/
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