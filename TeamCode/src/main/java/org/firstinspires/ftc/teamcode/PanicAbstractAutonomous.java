package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public abstract class PanicAbstractAutonomous extends LinearOpMode {

    // motors and servos
    public static final String LEFT_MOTOR = "Left Drive Train";
    public static final String RIGHT_MOTOR = "Right Drive Train";
    DcMotor motorRight;
    DcMotor motorLeft;

    public static final String COLLECTION_SYSTEM_MOTOR = "Collection System Motor";
    DcMotor collectionMotor;

    public static final String BUTTON_PUSHER = "Beacon Button Pusher";
    public static final String FLYWHEEL_LAUNCHER = "Flywheel Servo Launcher";
    Servo buttonPusher;
    Servo particleFlipper;
    CRServo loader;

    public static final String FRONT_FLYWHEEL_MOTOR = "Front Flywheel Motor";
    public static final String BACK_FLYWHEEL_MOTOR = "Back Flywheel Motor";
    public static final String SHOOTER = "Shooter";

    DcMotor frontFlywheelMotor;
    DcMotor backFlywheelMotor;
    protected VoltageSensor voltageSensor;

    // sensors
    public static final String BOTTOM_SENSOR = "Bottom Sensor";
    public static final String BEACON_SENSOR = "Beacon Sensor";
    public static final String BEACON_SENSOR_2 = "Beacon Sensor 2";
    public static final String RANGE_SENSOR = "Range Sensor";
    public static final String GYRO_SENSOR = "Gyro Sensor";
    public static final String LOADER = "Loader";

    protected OpticalDistanceSensor bottomSensor;
    protected ColorSensor beaconSensor;
    protected ColorSensor beaconSensor2;

    protected ModernRoboticsI2cRangeSensor rangeSensor;
    protected ModernRoboticsI2cGyro gyroSensor;

    // constant bank
    public static final int ENCODER_COUNTS = 28;
    public static final int MOTOR_COUNTS_40 = 40;

    public static final double PI = Math.PI;
    public static final double WHEEL_DIAMETER = 3.978; // :P
    public static final double SPIN_DIAMETER = 14.50;
    public static final double SPIN_CIRCUMFERENCE = SPIN_DIAMETER * PI; // 45.553093477052001957708329057553
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
    public static final double ENCODER_COUNTS_PER_IN = (ENCODER_COUNTS * MOTOR_COUNTS_40) / (WHEEL_CIRCUMFERENCE);
    public static final double ENCODER_COUNTS_PER_DEGREE_SPIN = ENCODER_COUNTS_PER_IN * (SPIN_CIRCUMFERENCE / 360);
    public static final double DISTANCE_PAST_WHITE_LINE = 0.5;

    // flywheel
    protected boolean lastDpadUp = false;
    protected boolean lastDpadDown = false;
    protected double flywheelSpeed = 0.0;
    protected boolean flywheelRunning = false;

    protected boolean failSafe = false;
    protected boolean parkProgram = false;

    // Servo Things
    protected void rightButton()
    {
        if (opModeIsActive())
        {
            long outTime = System.currentTimeMillis() + 373;

            while (System.currentTimeMillis() < outTime && opModeIsActive())
            {
                buttonPusher.setDirection(Servo.Direction.REVERSE);
                buttonPusher.setPosition(0);
            }
            bumpDriveTrain();
            buttonPusher.setPosition(0.5);
            outTime = System.currentTimeMillis() + 180;
            while (System.currentTimeMillis() < outTime && opModeIsActive())
            {
                buttonPusher.setDirection(Servo.Direction.FORWARD);
                buttonPusher.setPosition(0);
            }
            buttonPusher.setPosition(.5);
        }
    }

    protected void leftButton()
    {
        if (opModeIsActive())
        {
            long outTime = System.currentTimeMillis() + 373;
            while (System.currentTimeMillis() < outTime && opModeIsActive())
            {
                buttonPusher.setDirection(Servo.Direction.FORWARD);
                buttonPusher.setPosition(0);
            }
            buttonPusher.setPosition(.5);
            bumpDriveTrain();
            outTime = System.currentTimeMillis() + 180;
            while (System.currentTimeMillis() < outTime && opModeIsActive())
            {
                buttonPusher.setDirection(Servo.Direction.REVERSE);
                buttonPusher.setPosition(0);
            }
            buttonPusher.setPosition(.5);
        }
    }

    private void bumpDriveTrain()
    {
        setDriveTrainPower(.25);
        sleep(200);
        setDriveTrainPower(-.25);
        sleep(200);
        stopDriveTrain();
    }
    // Motor Count things
    int inchesToEncoderCount(double inches) {
        return ((int) (inches * ENCODER_COUNTS_PER_IN));
    }

    int degreesToEncoderCount(double degrees) {
        return ((int) (degrees * ENCODER_COUNTS_PER_DEGREE_SPIN));
    }

    // Drive train things
    protected void setDriveTrainPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    protected void stopDriveTrain() {
        setDriveTrainPower(0);
    }

    // flywheel stuff
    protected void stopFlywheel() {
        setFlywheelSpeed(0);
        flywheelRunning = true;
    }

    protected void setFlywheelSpeed(double power) {
        backFlywheelMotor.setPower(power);
        frontFlywheelMotor.setPower(power);
    }

    protected void startFlywheel() {
//        flywheelSpeed = 0.45;
        frontFlywheelMotor.setMaxSpeed(900);
        backFlywheelMotor.setMaxSpeed(900);
        setFlywheelSpeed(flywheelSpeed);
        flywheelRunning = true;
    }

    // drive train reset
    protected void resetDriveTrainEncoders() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void runDriveTrainUntil(double inches) {
        if (opModeIsActive())
        {
            double targetInEncoderCounts = inchesToEncoderCount(inches);
            if (motorLeft.getDirection().equals(DcMotorSimple.Direction.FORWARD))
            {
                while (motorLeft.getCurrentPosition() < targetInEncoderCounts && opModeIsActive()) {
                    telemetry.addData("Left Encoder: ", motorLeft.getCurrentPosition());
                    telemetry.update();
                    setDriveTrainPower(0.36);
                }
            }
            else
            {
                while (motorRight.getCurrentPosition() < targetInEncoderCounts && opModeIsActive()) {
                    telemetry.addData("Right Encoder: ", motorRight.getCurrentPosition());
                    telemetry.update();
                    setDriveTrainPower(0.36);
                }
            }
            stopDriveTrain();
        }
    }

    public void approachBeacon(double distance) {
        if (opModeIsActive())
        {
            buttonPusherIsForward();
            while(rangeSensor.cmUltrasonic() < 255 && rangeSensor.cmUltrasonic() > distance
                    && opModeIsActive())
            {
                setDriveTrainPower(0.13);
            }
            stopDriveTrain();
        }
    }

    public void pressRedButton() {
        if (isBeaconRed())
        {
            leftButton();
//            sleep(500);
        }
        else if (isBeaconBlue())
        {
            rightButton();;
//            sleep(500);
        }
        else
        {
            // we are not lined up so do nothing
        }
    }

    public void pressBlueButton() {
        if (isBeaconBlue())
        {
            leftButton();
//            sleep(500);
        }
        if (isBeaconRed())
        {
            rightButton();
//            sleep(500);
        }
        else
        {
            // we are not lined up so do nothing
        }
    }

    public void spinClockwise() {
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void spinCounterClockwise() {
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void buttonPusherIsForward() {
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void shooterIsForward() {
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void findWhiteLine(int distance) {
        if (opModeIsActive())
        {
            bottomSensor.enableLed(true);
            motorLeft.setTargetPosition(inchesToEncoderCount(distance));
            while (!isColorWhite() && opModeIsActive()) {
                if (motorLeft.getCurrentPosition() > motorLeft.getTargetPosition()) {
                  failSafe = true;
                  break;
                }
                setDriveTrainPower(0.30);
                telemetry.addData("Bottom Sensor ", bottomSensor.getLightDetected());
                telemetry.update();
            }
            stopDriveTrain();
        }
    }

    public boolean isBeaconRed() {
        return ((beaconSensor.red() > 3 /*|| beaconSensor2.red() > 3*/) && beaconSensor.blue() < 10 && beaconSensor.green() < 10);// ||
                //(beaconSensor2.red() > 3 && beaconSensor2.blue() < 10 && beaconSensor2.green() < 10);
    }

    public boolean isBeaconBlue() {
        return (beaconSensor.red() < 10 && (beaconSensor.blue() > 3 /*|| beaconSensor2.blue() > 3*/) && beaconSensor.green() < 10);// ||
               // (beaconSensor2.red() < 10 && beaconSensor2.blue() > 3 && beaconSensor2.green() < 10);

    }

    public boolean isColorWhite() {
        return bottomSensor.getLightDetected() > .1;
    }

    protected enum Direction {
        CCW, CW // directions for the next
    }

    public void flywheelSpeed() {
        double voltage = voltageSensor.getVoltage();
        if (voltage >= 13.5) {
            flywheelSpeed = 0.35;
        }
        else if (voltage >= 13.2) {
            flywheelSpeed = 0.37;
        }
        else if (voltage >= 13.0) {
            flywheelSpeed = 0.39;
        }
        else if (voltage >= 12.7) {
            flywheelSpeed = 0.41;
        }
        else {
            flywheelSpeed = 0.45;
        }
        if (parkProgram) {
            flywheelSpeed += 0.05;
        }
    }
    public void cycleBall() {
        long outTime = System.currentTimeMillis() + 655;
        while (System.currentTimeMillis() < outTime)
        {
            loader.setDirection(CRServo.Direction.REVERSE);
            loader.setPower(0.5);
        }
        loader.setPower(0);
    }
    //protected
    public void initRobot() { // Self made init
        buttonPusher = hardwareMap.servo.get(BUTTON_PUSHER);
        buttonPusher.setPosition(0.5);

        motorRight = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        motorLeft = hardwareMap.dcMotor.get(LEFT_MOTOR);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        resetDriveTrainEncoders();

        frontFlywheelMotor = hardwareMap.dcMotor.get(FRONT_FLYWHEEL_MOTOR);
        frontFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backFlywheelMotor = hardwareMap.dcMotor.get(BACK_FLYWHEEL_MOTOR);
        backFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.get(SHOOTER);
        flywheelSpeed();

        particleFlipper = hardwareMap.servo.get(FLYWHEEL_LAUNCHER);
        particleFlipper.setPosition(0.46);
        loader = hardwareMap.crservo.get(LOADER);

        bottomSensor = hardwareMap.opticalDistanceSensor.get(BOTTOM_SENSOR);
        //bottomSensor.setI2cAddress(I2cAddr.create7bit(0x1e));
        bottomSensor.enableLed(false);

        beaconSensor = hardwareMap.colorSensor.get(BEACON_SENSOR);
        beaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));
        beaconSensor.enableLed(false);

        beaconSensor2 = hardwareMap.colorSensor.get(BEACON_SENSOR_2);
        beaconSensor2.setI2cAddress(I2cAddr.create7bit(0x1e));
        beaconSensor2.enableLed(false);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGE_SENSOR);

        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRO_SENSOR);
        gyroSensor.calibrate();
    }
}