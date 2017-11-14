package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.GyroSensor;

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

    import static java.lang.Thread.sleep;

@Autonomous(name = "RelicBlueFront", group = "RiderModes")
public class CyberRelicBlueFront extends CyberRelicAbstract{

    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;

    @Override
    public void init() {

        super.init();

        lightSensor.enableLed(true);

    }

    @Override
    public void loop()
    {

        super.loop();

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
                seqRobot++;
                break;
            }

            case 2: {
                servoGem.setPosition(tbd);
                seqRobot++;
                break;
            }

            case 3: {
                lightSensor.getLightDetected();

                if (red) {
                    motorLeftA.setTargetPosition(-GEM_ROTATE);
                    motorLeftB.setTargetPosition(-GEM_ROTATE);
                    motorRightA.setTargetPosition(GEM_ROTATE);
                    motorRightB.setTargetPosition(GEM_ROTATE);
                    motorLeftA.setPower(.25);
                    motorLeftB.setPower(.25);
                    motorRightA.setPower(.25);
                    motorRightB.setPower(.25);
                }

                if (blue) {
                    motorLeftA.setTargetPosition(GEM_ROTATE);
                    motorLeftB.setTargetPosition(GEM_ROTATE);
                    motorRightA.setTargetPosition(-GEM_ROTATE);
                    motorRightB.setTargetPosition(-GEM_ROTATE);
                    motorLeftA.setPower(.1);
                    motorLeftB.setPower(.1);
                    motorRightA.setPower(.1);
                    motorRightB.setPower(.1);
                }

                seqRobot++;
                break;
            }

            case 4: {
                servoGem.setPosition(tbd);
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot++;
                break;
            }

            case 5: {
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

            case 6: {
                motorLeftA.setTargetPosition(DECRIPT_ROTATE);
                motorLeftB.setTargetPosition(DECRIPT_ROTATE);
                motorRightA.setTargetPosition(-DECRIPT_ROTATE);
                motorRightB.setTargetPosition(-DECRIPT_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot++;
                break;
            }

            case 7:
            {

                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

                parameters.vuforiaLicenseKey = "AdlfowT/////AAAAGUpRoaEvyUfNtuTDeKMo6qEf2Y8oPuvPan17xUGgdDWoYKTx+JNrzPv2tPPmKMQcyOw9MNnOeGDXCPFCDOOKjsUTjil2cGK9odRVmSWL0xsxdsxtbz9Y3ZW2q1fi9IJsBofvfxfTa/6t9JDldr1+6lcBi9izU2k0ZC9Md6S8DHkcvQ7Q7P9NRepmQZXU+ztVWxB9gNHJ1128u3zADXS+pIkW9qIUHfc6UybysSNpmeh65VxdFRu2Tnlwh3fqAB+NjG9eMmgP49FyW3C3wnkwMMCVqT4JBdhRPviRHyp7lXXtzVqr/BB30ww0hk0W7gyiANIbVvQq/04i18SFFuS5H9zX2v0g5J3ViTvfUybve/AA";

                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                VuforiaTrackable relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                relicTrackables.activate();

                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                    {


                        telemetry.addData("VuMark", "%s visible", vuMark);

                        if (vuMark != RelicRecoveryVuMark.CENTER && vuMark != RelicRecoveryVuMark.RIGHT )
                        {
                            leftCol = true;
                            centerCol = false;
                            rightCol = false;
                        }
                        else if (vuMark != RelicRecoveryVuMark.LEFT && vuMark != RelicRecoveryVuMark.RIGHT )
                        {
                            leftCol = false;
                            centerCol = true;
                            rightCol = false;
                        }
                        else if (vuMark != RelicRecoveryVuMark.LEFT && vuMark != RelicRecoveryVuMark.CENTER )
                        {
                            leftCol = false;
                            centerCol = false;
                            rightCol = true;
                        }



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
                    }
                    else
                    {
                        telemetry.addData("VuMark", "not visible");
                    }


                    telemetry.update();
                seqRobot++;
                break;
            }

            case 8:
            {
                motorLeftA.setTargetPosition(0);
                motorLeftB.setTargetPosition(0);
                motorRightA.setTargetPosition(0);
                motorRightB.setTargetPosition(0);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot++;
                break;
            }

            case 9:
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
            case 10:
            {

                if (leftCol)
                {
                    if (rangeSensor.cmUltrasonic() < tbd) {
                        motorLeftA.setPower(.25);
                        motorLeftB.setPower(.25);
                        motorRightA.setPower(.25);
                        motorRightB.setPower(.25);
                    } else if (rangeSensor.cmUltrasonic() >= tbd) {
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    }
                }

                if (centerCol)
                {
                    if (rangeSensor.cmUltrasonic() < tbd) {
                        motorLeftA.setPower(.25);
                        motorLeftB.setPower(.25);
                        motorRightA.setPower(.25);
                        motorRightB.setPower(.25);
                    } else if (rangeSensor.cmUltrasonic() >= tbd) {
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    }
                }

                if (rightCol)
                {
                    if (rangeSensor.cmUltrasonic() < tbd) {
                        motorLeftA.setPower(.25);
                        motorLeftB.setPower(.25);
                        motorRightA.setPower(.25);
                        motorRightB.setPower(.25);
                    } else if (rangeSensor.cmUltrasonic() >= tbd) {
                        motorLeftA.setPower(0);
                        motorLeftB.setPower(0);
                        motorRightA.setPower(0);
                        motorRightB.setPower(0);
                    }
                }

                seqRobot++;
                break;
            }

            case 11:
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

            case 12:
            {
                motorLeftA.setTargetPosition(-GLYPH_ROTATE);
                motorLeftB.setTargetPosition(-GLYPH_ROTATE);
                motorRightA.setTargetPosition(GLYPH_ROTATE);
                motorRightB.setTargetPosition(GLYPH_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot++;
                break;
            }

            case 13:
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

            case 14:
            {

                targetDrRotateDeg = 0f;
                targetDrDistInch = 5f; // Set target distance
                targetPower = 0.1d;  // Set power

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);


                seqRobot++;
                break;
            }

            case 15:
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

            case 16:
            {
                servoGlyph1.setPosition(tbd);
                servoGlyph2.setPosition(tbd);
                seqRobot++;
                break;
            }

            case 17:
            {
                targetDrRotateDeg = 0f;
                targetDrDistInch = -5f; // Set target distance
                targetPower = 0.1d;  // Set power

                targetPosLeftA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftA);
                targetPosLeftB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorLeftB);
                targetPosRightA = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightA);
                targetPosRightB = cmdMoveR(targetDrDistInch, (float)ENCODER_CNT_PER_IN_DRIVE, targetPower, motorRightB);


                seqRobot++;
                break;
            }

            case 18:
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

            case 19:
            {
                motorLeftA.setTargetPosition(END_ROTATE);
                motorLeftB.setTargetPosition(END_ROTATE);
                motorRightA.setTargetPosition(-END_ROTATE);
                motorRightB.setTargetPosition(-END_ROTATE);
                motorLeftA.setPower(.1);
                motorLeftB.setPower(.1);
                motorRightA.setPower(.1);
                motorRightB.setPower(.1);
                seqRobot++;
                break;
            }

            case 20:
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












        }






    }