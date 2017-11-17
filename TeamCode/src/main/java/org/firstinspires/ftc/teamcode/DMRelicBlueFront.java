package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "DMRelicBlueFront", group = "RiderModes")
public class DMRelicBlueFront extends DMRelicAbstract{

    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;

    @Override
    public void init() {

        super.init();

        //lightSensor.enableLed(true);

    }

    @Override
    public void loop()
    {

        super.loop();

        telemetry.addData("Entering switch - seq = ", seqRobot);
        telemetry.update();

        switch (seqRobot) {

            case 1: {
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
                seqRobot = 4;
                break;
            }

            case 4: {
                sGem.setPosition(0.8);
                seqRobot+=2;
                break;
            }

            case 6: {
                //lightSensor.getLightDetected();

                //telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", snDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", snColor.alpha());
                telemetry.addData("Red  ", snColor.red());
                telemetry.addData("Green", snColor.green());
                telemetry.addData("Blue ", snColor.blue());
                if (snColor.red() > snColor.blue()) {  //move forward
                    telemetry.addData("Gem ", "RED");
                    targetPower = 0.1d;  // Set power
                }
                else {  //move back
                    telemetry.addData("Gem ", "BLUE");
                    targetPower = -0.1d;  // Set power
                }

                targetDrRotateDeg = 0f;
                targetDrDistInch = 2f; // Set target distance

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);

                /*
                //Move back to start position
                targetPower *= -1;  // Set power

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                */

                seqRobot+=2;
                break;
            }

            case 8: {  // Move robot back to starting position
                sGem.setPosition(0);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot+=2;
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
                seqRobot+=2;
                break;
            }

            case 12: { // Rotate to face decryption key
                motorLeftA.setTargetPosition(DECRIPT_ROTATE);
                motorLeftB.setTargetPosition(DECRIPT_ROTATE);
                motorRightA.setTargetPosition(-DECRIPT_ROTATE);
                motorRightB.setTargetPosition(-DECRIPT_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
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
                seqRobot+=2;
                break;
            }

            case 16: // Reset position after vuforia
            {
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot+=2;
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
                seqRobot++;
                break;
            }
            case 20:  // Move robot to correct column
                        //28", 35", 42" - 9"
            {

                if (leftCol)
                {
                    targetDrRotateDeg = 0f;
                    targetDrDistInch = 19f; // Set target distance
                    targetPower = 0.1d;  // Set power

                    targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                    targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                    targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                    targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                } else if (centerCol)
                {
                    targetDrRotateDeg = 0f;
                    targetDrDistInch = 26f; // Set target distance
                    targetPower = 0.1d;  // Set power

                    targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                    targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                    targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                    targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                } else if (rightCol)
                {
                    targetDrRotateDeg = 0f;
                    targetDrDistInch = 33f; // Set target distance
                    targetPower = 0.1d;  // Set power

                    targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                    targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                    targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                    targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);
                }

                seqRobot+=2;
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
                seqRobot++;
                break;
            }

            case 24:  // Rotate to Glyph
            {
                motorLeftA.setTargetPosition(-GLYPH_ROTATE);
                motorLeftB.setTargetPosition(-GLYPH_ROTATE);
                motorRightA.setTargetPosition(GLYPH_ROTATE);
                motorRightB.setTargetPosition(GLYPH_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot+=2;
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
                seqRobot+=2;
                break;
            }

            case 28: // move forward 5 "
            {

                targetDrRotateDeg = 0f;
                targetDrDistInch = 5f; // Set target distance
                targetPower = 0.1d;  // Set power

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);


                seqRobot+=2;
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
                seqRobot+=2;
                break;
            }

            case 32:  // move and open Glyph
            {
                sGlyphL.setPosition(0.05);
                sGlyphR.setPosition(0.83);
                seqRobot+=2;
                break;
            }

            case 34:  // move back
            {
                targetDrRotateDeg = 0f;
                targetDrDistInch = -5f; // Set target distance
                targetPower = 0.1d;  // Set power

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);


                seqRobot+=2;
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
                seqRobot++;
                break;
            }

            case 38:
            {
                motorLeftA.setTargetPosition(END_ROTATE);
                motorLeftB.setTargetPosition(END_ROTATE);
                motorRightA.setTargetPosition(-END_ROTATE);
                motorRightB.setTargetPosition(-END_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot+=2;
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




        }
        telemetry.update();












        }






    }