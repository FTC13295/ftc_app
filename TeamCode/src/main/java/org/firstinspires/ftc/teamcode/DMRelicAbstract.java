
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public abstract class DMRelicAbstract extends OpMode {

    protected ModernRoboticsI2cRangeSensor
            rangeSensor;
    // Set Servos
    protected Servo
            sGlyphL, sGlyphR, sGem, sRelic;

    protected ColorSensor
            snColor;

    protected DistanceSensor
            snDistance;

    protected VuforiaLocalizer
            vuforia;

    //BNO055IMU imu;

    protected CRServo
            sGLift, sBArm;


    protected DcMotor
            motorLeftA, motorLeftB,
            motorRightA, motorRightB,
            motorGlyphLift,
            motorRelicExtend, motorRelicLift;

    protected boolean                  // Used to detect initial press of "A" button on gamepad 1
            pulseCaseMoveDone,                          // Case move complete pulse
            red,
            blue,
            fieldOrient,
            bDirection,
            grabbed,
            debug,                      // Flag for debugging
            Gdown, Gopen,               // Flag for Glyph lift down and Glygh open or closed
            bArmDown = false,           // Flag for Back Arm
            slowdown = false,
            leftCol, rightCol, centerCol,
            relicopen = false;          // Flg for Relic jaw

    protected float
            targetDrDistInch,                   // Targets for motor moves in sequence (engineering units)
            targetDrRotateDeg,
            drivepower,
            temp_x_stick, temp_y_stick,          // Temporary x and y stick value
            targetPower,                        // General motor power variable (%, -1.0 to 1.0)
            hsvValues[] = {0F, 0F, 0F};
    // Auto: Values used to determine current color detected

    protected double
            temp, gyro,
            x,y,
            glyphL, glyphR,
            spos,
            powerLeftA, powerLeftB,
            powerRightA, powerRightB,
            velocityDrive, strafeDrive, rotationDrive,
            throttleLift;


    // Establish Integer Variables
    protected int
            seqRobot, target,                               // Switch operation integer used to identify sequence step.
            targetPosLeftA, targetPosLeftB,
            targetPosRightA, targetPosRightB,      // Drive train motor target variables (encoder counts)
            IncVal;

    // Establish Integer Constants
    final static int
            DECRIPT_ROTATE = 0,
            SLEEP_TIME = 1000,                   // Default wait time = 1 sec
            ERROR_DRV_POS = 20,                 // Allowed error in encoder counts following drive train position move
            GLYPH_ROTATE = 2040,                // need to confirm # of encoder rotations for 90 deg
                                                // full rotation of the wheel = 1120
                                                // 2240 = 95 deg
            END_ROTATE = 4080;                     // final position - left facing cypher
    // Establish Float Constants
    final static float
            PowerRatio = 0.7f,
            DEFAULT_MOVE_SPEED = 0.5f,                   //Default move speed for autonomous
            SLOW_POWER = 0.4f,
            REG_POWER = 0.7f,
            FULL_POWER = 1.0f,
            GEM_DISTANCE = 3.0f,                        //Set distance to 3 inches
            ENCODER_CNT_PER_IN_DRIVE = 89.12677f;       //59.41979167d; // (28 count/motor rev x 40 motor rev / shaft rev) / (6" dia. wheel x pi)
                                                        //Ticks per rev - 1120 (AndyMark)
                                                        //1120 / Diam * PI = 1120 / 4 * 3.1415 = 89.12677

    // Establish Double Constants
    final static double
            DELAY_DRV_MOV_DONE = 0.1d;        // Hold/wait 0.1s after drive train move complete (seconds)


    // Establish Controller and Device String Constants
    // These names need to match the Robot Controller configuration file device names.
    final static String

            MOTOR_DRIVE_LEFT_A = "leftA",
            MOTOR_DRIVE_LEFT_B = "leftB",
            MOTOR_DRIVE_RIGHT_A = "rightA",
            MOTOR_DRIVE_RIGHT_B = "rightB",
            MOTOR_GLYPH_LIFT = "mGL",
            SENSOR_GYRO = "gyro",
            //SENSOR_RANGE = "range",
            GLYPH_LEFT = "sGlyphL",
            GLYPH_RIGHT = "sGlyphR",
            Gem = "sGem",
            Servo_GlyphLift = "sGLift",
            Servo_BackArm = "sBArm",
            //GLYPH_LIFT = "gLift";
            Sensor_Color_Distance = "snCD",
            Servo_Relic = "sRelic",
            MOTOR_RELIC_EXTEND = "mRE",
            MOTOR_RELIC_LIFT = "mRL";



    //------------------------------------------------------------------
    // Robot Initialization Method
    //------------------------------------------------------------------
    @Override
    public void init() {
        // Get references to dc motors and set initial mode and direction
        // It appears all encoders are reset upon robot startup, but just in case, set all motor
        // modes to Stop-And-Reset-Encoders during initialization.
        motorLeftA = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT_A);
        motorLeftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftA.setDirection(DcMotor.Direction.FORWARD);

        motorLeftB = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT_B);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setDirection(DcMotor.Direction.FORWARD);

        motorRightA = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT_A);
        motorRightA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightA.setDirection(DcMotor.Direction.REVERSE);

        motorRightB = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT_B);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setDirection(DcMotor.Direction.REVERSE);

        motorGlyphLift = hardwareMap.dcMotor.get(MOTOR_GLYPH_LIFT);
        motorGlyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGlyphLift.setDirection(DcMotor.Direction.FORWARD);


        motorRelicExtend = hardwareMap.dcMotor.get(MOTOR_RELIC_EXTEND);
        motorRelicExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRelicExtend.setDirection(DcMotor.Direction.FORWARD);

        motorRelicLift = hardwareMap.dcMotor.get(MOTOR_RELIC_LIFT);
        motorRelicLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRelicLift.setDirection(DcMotor.Direction.FORWARD);

        sGlyphL = hardwareMap.servo.get(GLYPH_LEFT);
        sGlyphR = hardwareMap.servo.get(GLYPH_RIGHT);
        sGem = hardwareMap.servo.get(Gem);

        sGLift = hardwareMap.crservo.get(Servo_GlyphLift);
        sBArm = hardwareMap.crservo.get(Servo_BackArm);

        sRelic = hardwareMap.servo.get(Servo_Relic);

        /*
        BNO055IMU.Parameters iparameters = new BNO055IMU.Parameters();
        iparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(iparameters);

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(iparameters);
*/

        //servoGlyph1.setPosition(180);
        //servoGlyph2.setPosition(180);

        //bDirection = true;

        // get a reference to the color sensor.
        snColor = hardwareMap.get(ColorSensor.class, Sensor_Color_Distance);

        // get a reference to the distance sensor that shares the same name.
        snDistance = hardwareMap.get(DistanceSensor.class, Sensor_Color_Distance);

        //set starting case #
        seqRobot = 1;



    } // End OpMode Initialization Method

    //------------------------------------------------------------------
    // Loop Method
    //------------------------------------------------------------------
    @Override
    public void loop()
    {
    }



    //------------------------------------------------------------------
    // Stop Method
    //------------------------------------------------------------------
    @Override
    public void stop()
    {    // stop all the motors when the program is stopped
        motorRightA.setPower(0);
        motorRightB.setPower(0);
        motorLeftA.setPower(0);
        motorLeftB.setPower(0);
        motorGlyphLift.setPower(0);
        motorRelicExtend.setPower(0);
        motorRelicLift.setPower(0);
    } // End OpMode Stop Method


    //------------------------------------------------------------------
    // Miscellaneous Methods
    //------------------------------------------------------------------

    // calcRotate Method
    // Calculate linear distance needed for desired rotation
    // Parameters:
    // 		rotateDeg = Rotation desired (degrees)
    //		anglePerIn = Angle of rotation when left and right move 1 inch in opposite directions
    // Return: linear distance in inches
    float calcRotate(float rotateDeg, float anglePerIn)
    {
        return rotateDeg / anglePerIn;
    }


    // cmdMoveR Method
    // Convert desired distance from inches to encoder counts, establish new motor target, and set
    // motor power. New motor target is assumed to be relative; in other words, motor target is
    // current position plus new distance.
    // Parameters:
    //		distIn = Relative target distance (inches)
    //		encoderCntPerIn = encoder-to-inches conversion
    //		power = desired motor power (%)
    //		motor = motor
    // Return: New target (encoder counts)

    int cmdMoveR(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = ((int) (distIn * encoderCntPerIn));// + motor.getCurrentPosition();

        // Set motor target and power
        motor.setPower(power);
        motor.setTargetPosition(target);

        return target;
    }

    // control glyph lift (servo)
    public void gliftUp ()
    {
        sGLift.setPower(0.9);

    }

    public void gliftDown()
    {
        sGLift.setPower(-0.9);

    }

    public void gliftStop()
    {
        sGLift.setPower(0);

    }

    // control glyph lift (motor)


    // control back arm
    public void lowerbarm(int climit)
    {
        for(int counter = 0; counter < climit; counter++)
        {
            sBArm.setPower(0.9);
        }
        sBArm.setPower(0);

    }

    public void initbarm()
    {
        sBArm.setPower(0.9);
        sBArm.setPower(0);
    }

    //Test Back Arm
    public void barmUp ()
    {
        sBArm.setPower(0.5);

    }

    public void barmDown()
    {
        sBArm.setPower(-0.5);

    }

    public void barmStop()
    {
        sBArm.setPower(0);

    }
    /*
    public void MvFrwdDist(double power, int distance)
    {
        // reset encoders
        motorRightA.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorRightB.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorLeftA.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorLeftB.setMode(DcMotor.RunMode.RESET_ENCODERS);

        // set targt position
        motorRightA.setTargetPosition(distance);
    }*/

    // cmdMoveA Method
    // Convert desired distance from inches to encoder counts, establish new motor target, and set
    // motor power. New motor target is assumed to be absolute; in other words, motor target is
    // based on the original home position.
    // Parameters:
    //		distIn = Absolute target distance (inches)
    //		encoderCntPerIn = encoder-to-inches conversion
    //		power = desired motor power (%)
    //		motor = motor
    // Return: New target (encoder counts)

    int cmdMoveA(float distIn, float encoderCntPerIn, float power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = (int) (distIn * encoderCntPerIn);

        // Set motor target and power
        motor.setTargetPosition(target);
        motor.setPower(power);

        return target;
    }

    // chkMove method
    // Verify motor has achieved target
    // Parameters:
    //		motor = motor
    //		target = desired target (encoder counts)
    //		delta = allowed +/- error from target (encoder counts)
    // Return:
    //		True if move complete
    //		False if move not complete

    boolean chkMove(DcMotor motor, int target, int delta)
    {
        int currentPos = motor.getCurrentPosition();
        return ((currentPos >= (target - delta)) && (currentPos <= (target + delta)));
    }



    //------------------------------------------------------------------
    // Miscellaneous Methods
    //------------------------------------------------------------------

    //	scaleInput method
    // (written by unknown FTC engineer)
    // 	This method scales the joystick input so for low joystick values, the
    //	scaled value is less than linear.  This is to make it easier to drive
    //	the robot more precisely at slower speeds.

    static double scaleInput(double dVal)
    {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0)
        {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16)
        {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        }
        else
        {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


    // limit method - Recommend not using this method
    // This method prevents over-extended motor movement. Once a limit is reached, you cannot go
    // any further, but you may reverse course. Unfortunately this does not prevent significant
    // overshoot. A better way to limit motor distance is to place the motor into Run-To-Position
    // mode, and then adjust power manually.
    //
    // Method Parameters:
    //     powerValue = desired motor throttle value
    //     direction = (once the encoders are wired properly, this term may go away)
    //     lower limit / upper limit = allowed range of movement
    //
    //  Motor Output:
    //      powerValue = recalculated motor throttle
    //
    //  If time permits, you can revise the code to reduce motor power as a limit is approached.

    float limit(float powerValue, double currentPos, double lowerLimit, double upperLimit)
    {
        if (currentPos > upperLimit)
        {
            if (powerValue > 0)
            {
                powerValue = 0;
            }
        }

        if (currentPos < lowerLimit)
        {
            if (powerValue < 0)
            {
                powerValue = 0;
            }
        }

        return powerValue;
    }

    //Reset motor encoder and set target position
    void resetME(int position) {
        motorLeftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftA.setTargetPosition(position);
        motorLeftB.setTargetPosition(position);
        motorRightA.setTargetPosition(position);
        motorRightB.setTargetPosition(position);
    }

    //Reset glyphlift motor encoder and set target position
    void resetMEG(int position) {
        motorGlyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlyphLift.setTargetPosition(position);
    }

    // move motors back to target position using provided power
    void movebackME(int position, float power) {
        motorLeftA.setTargetPosition(position);
        motorLeftB.setTargetPosition(position);
        motorRightA.setTargetPosition(position);
        motorRightB.setTargetPosition(position);
        motorLeftA.setPower(power);
        motorLeftB.setPower(power);
        motorRightA.setPower(power);
        motorRightB.setPower(power);
    }

    // move Glyphlift motor back to target position using provided power
    void movebackMEG(int position, float power) {
        motorGlyphLift.setTargetPosition(position);
        motorGlyphLift.setPower(power);
    }
}