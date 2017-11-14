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

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.hardware.DcMotor;
    //import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
    import com.qualcomm.robotcore.hardware.GyroSensor;

    /* ------------------------------------------------------------------
     * This Op Mode is the super class used for Robot Control
     *
     * Control Modules
     *  1. Motor Controller Module
     *  2. Motor Controller Module
     *  4. Servo Controller Module
     *  5. Core Device Interface Module
     *
     * Gamepads
     * 	Not used; however, for troubleshooting prior to competition, Gamepad1's "A" button will allow
     * 	the programmer to advance the sequence 1 step.
     * ------------------------------------------------------------------
     */
    public abstract class CyberVortexAbstract extends OpMode {

        // Set Servos
        protected Servo

                servoButtonL, servoButtonR; //Left and Right button-pushers. NOTE: These are Vex Motors.


        // Therefore, they operate in a different fashion. See the leftButton and rightButton methods for more detail.

        //protected CRServo
        //servoCycle;

        protected ColorSensor
                colorSensorB, colorSensorL;

        protected GyroSensor
                gyroSensor;

        protected CRServo
                servoLoad, servoRelease;


        // Set DcMotors
        protected DcMotor
                motorRight, motorLeft,  // Drive Train Motors Left and Right
                motorCollect, motorFlicker,   // Particle Handling Motors Collection and Launcher
                motorSpool,  // Capping Mechanism Motor Spool
                motorLeftA, motorLeftB,
                motorRightA, motorRightB,
                motorTest;

        //DeviceInterfaceModule cdim;

        // Set Boolean Variables
        protected boolean pulseGamePad1A,                   // Used to detect initial press of "A" button on gamepad 1
                pulseCaseMoveDone,                          // Case move complete pulse
                white,                                      // Variables used to detect current color, used for autonomous control.
                red,
                blue,
                leftOut,
                rightOut,
                whiteLine;

        // Establish Float Variables
        protected float targetDrDistInch,                   // Targets for motor moves in sequence (engineering units)
                targetDrRotateDeg,
                powerLeft, powerRight, // TeleOP: Drive train power (or speed in Run-With-Encoders mode)
                powerLeftA, powerLeftB,
                powerRightA, powerRightB,
                throttleALength, throttleAJoint, //TeleOp: Arm Powers
                throttleDrive, directionDrive,   // TeleOP: Power and direction variables used to calculate Left/Right drive motor power
                velocityDrive, strafeDrive, rotationDrive,
                hsvValues[] = {0F, 0F, 0F};           // Auto: Values used to determine current color detected

        // Establish Double Variables
        protected double targetPower;                         // General motor power variable (%, -1.0 to 1.0)

        // Establish Integer Variables
        protected int seqRobot, target,                               // Switch operation integer used to identify sequence step.
                targetPosLeft, targetPosRight;      // Drive train motor target variables (encoder counts)

        // Establish Integer Constants
        final static int
                COLLECT_RUN_TIME = 1500,
                RED_ROTATE_COUNTS = 790,
                RED_ROTATE_COUNTS_MIN = 775,
                RED_ROTATE_COUNTS_MAX = 820,
                BLUE_ROTATE_COUNTS_2 = 200,
                BLUE_ROTATE_COUNTS_2_MIN = 185,
                BLUE_ROTATE_COUNTS_2_MAX = 230,
                BLUE_ROTATE_COUNTS = 1287,
                BLUE_ROTATE_COUNTS_MIN = 1272,
                BLUE_ROTATE_COUNTS_MAX = 1302,
                ROTATE_COUNTS = 1317,
                ROTATE_COUNTS_MIN = 1302,
                ROTATE_COUNTS_MAX = 1332,
                COUNTS_PER_INCH = 2215,
                LAUNCH_ROTATE = 2215,               //The amount of encoder counts for one 360 degree rotation of the flicker system.
                ERROR_DRV_POS = 20,                 // Allowed error in encoder counts following drive train position move
                ERROR_ARM_EXT_POS = 20,             // Allowed error in encoder counts following arm extend move
                ERROR_ARM_PIV_POS = 20,             // Allowed error in encoder counts following arm pivot move
                LIMIT_ARM_EXT_MAX = 8150,           // Maximum limit for extension arm (encoder counts)
                LIMIT_ARM_EXT_MIN = 0,              // Minimum limit for extension arm (encoder counts)
                LIMIT_ARM_EXT_MIN_FOR_ROTATE = 1000,// Minimum value of arm Extension before arm rotation allowed (encoder counts)
                LIMIT_ARM_PIV_MAX = 2000,           // Maximum limit for arm rotation (encoder counts)
                LIMIT_ARM_PIV_MIN = 0,              // Minimum limit for arm rotation (encoder counts)
                LIMIT_ARM_PIV_MAX_TO_RETRACT = 20,  // Minimum value of Arm rotation before arm can fully retract (encoder counts)
                PUSHER_RUN_TIME = 500,
                CYCLE_TIME = 250,
                PUSHER_SLEEP_TIME = 500,
                RELEASE_RUN_TIME = 250;


        // Establish Float Constants
        final static float
                ENCODER_CNT_PER_IN_DRIVE = 81.49f,  // (28 count/motor rev x 40 motor rev / shaft rev) / (4 3/8" dia. wheel x pi)
                DEGREE_PER_IN_TRAVEL = 7.5f,        // Amount of rotation for distance travelled (-Left:Right); determined by experiment.
                ENCODER_CNT_PER_IN_ARM_EXT = 40.0f, // NOTICE - This value still needs to be calculated!
                ENCODER_CNT_PER_DEG_ARM_PIV = 30.0f;// NOTICE - This value still needs to be calculated!

        // Establish Double Constants
        final static double
                DELAY_DRV_MOV_DONE = 0.1d,          // Hold/wait 0.1s after drive train move complete (seconds)
                DELAY_ARM_MOV_DONE = 0.1d,          // Hold/wait 0.1s after arm move complete (seconds)
                STOP_COLOR_SEARC
                        = 3.0d;

        // Establish Controller and Device String Constants
        // These names need to match the Robot Controller configuration file device names.
        final static String
                MOTOR_COLLECTION = "collect",
                MOTOR_DRIVE_LEFT = "left",
                MOTOR_DRIVE_RIGHT = "right",
                MOTOR_DRIVE_LEFT_A = "leftA",
                MOTOR_DRIVE_LEFT_B = "leftB",
                MOTOR_DRIVE_RIGHT_A = "rightA",
                MOTOR_DRIVE_RIGHT_B = "rightB",
                MOTOR_FLICK = "flick",
                MOTOR_SPOOL = "spool",
                MOTOR_TEST = "test",
                SERVO_LOCK = "lock",
                SERVO_LOAD = "load",
                SERVO_BUTTON_L = "buttonl",
                SERVO_BUTTON_R = "buttonr",
                SERVO_RELEASE = "release",
        //SERVO_CYCLE = "cycle",
        SENSOR_COLOR_BEACON = "beacon",
                SENSOR_COLOR_LINE = "line",
                SENSOR_GYRO = "gyro";

        //------------------------------------------------------------------
        // Robot Initialization Method
        //------------------------------------------------------------------
        @Override
        public void init() {
            // Get references to dc motors and set initial mode and direction
            // It appears all encoders are reset upon robot startup, but just in case, set all motor
            // modes to Stop-And-Reset-Encoders during initialization.
            motorLeft = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT);        // Drive train left motor
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft.setDirection(DcMotor.Direction.FORWARD);

            motorRight = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT);    // Drive train right motor
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setDirection(DcMotor.Direction.REVERSE);

            motorLeftA = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT_A);
            motorLeftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftA.setDirection(DcMotor.Direction.FORWARD);

            motorLeftB = hardwareMap.dcMotor.get(MOTOR_DRIVE_LEFT_B);
            motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftB.setDirection(DcMotor.Direction.FORWARD);

            motorRightA = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT_A);
            motorRightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightA.setDirection(DcMotor.Direction.REVERSE);

            motorRightB = hardwareMap.dcMotor.get(MOTOR_DRIVE_RIGHT_B);
            motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightB.setDirection(DcMotor.Direction.REVERSE);

            motorCollect = hardwareMap.dcMotor.get(MOTOR_COLLECTION);    // Collection motor
            motorCollect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorCollect.setDirection(DcMotor.Direction.FORWARD);

            motorFlicker = hardwareMap.dcMotor.get(MOTOR_FLICK);    // Launcher motor
            motorFlicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFlicker.setDirection(DcMotor.Direction.FORWARD);

            motorSpool = hardwareMap.dcMotor.get(MOTOR_SPOOL);      //Spool motor
            motorSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSpool.setDirection(DcMotor.Direction.REVERSE);

            motorTest = hardwareMap.dcMotor.get(MOTOR_TEST);
            motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorTest.setDirection(DcMotor.Direction.FORWARD);

            servoButtonL = hardwareMap.servo.get(SERVO_BUTTON_L);       //Left Servo Button
            servoButtonR = hardwareMap.servo.get(SERVO_BUTTON_R);       //Right Servo Button

            servoRelease = hardwareMap.crservo.get(SERVO_RELEASE);

            servoLoad = hardwareMap.crservo.get(SERVO_LOAD);


            seqRobot = 1;    // Set seqRobot = 1 to kick off the sequence.

            //get a reference to our ColorSensor objects.
            colorSensorB = hardwareMap.colorSensor.get(SENSOR_COLOR_BEACON);    //Beacon Color Sensor
            colorSensorL = hardwareMap.colorSensor.get(SENSOR_COLOR_LINE);      //White Line Color Sensor

            //get a reference to our GyroSenror object.
            gyroSensor = hardwareMap.gyroSensor.get(SENSOR_GYRO);       //Gyro Sensor

            //cdim.setDigitalChannelState(LED_CHANNEL,true);

            whiteLine = false;      //White line not yet detected

            servoButtonL.setPosition(.5);
            servoButtonR.setPosition(.5);
            servoRelease.setPower(0);
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
            motorRight.setPower(0);
            motorLeft.setPower(0);
            motorRightA.setPower(0);
            motorRightB.setPower(0);
            motorLeftA.setPower(0);
            motorLeftB.setPower(0);
            motorCollect.setPower(0);
            motorFlicker.setPower(0);
            motorTest.setPower(0);
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

    public void releaseOut()
    {

            servoRelease.setDirection(CRServo.Direction.FORWARD);
            servoRelease.setPower(1);

    }

    public void releaseLock()
    {
            servoRelease.setDirection(CRServo.Direction.REVERSE);
            servoRelease.setPower(1);

    }

    public void rightButtonIn()
    {
        //Set the amount of time we need the servo to run out for to the current time + PUSHER_RUN_TIME constant.
        long inTime = System.currentTimeMillis() + PUSHER_RUN_TIME + 250;

        //While the timer hasn't reached outTime, have the servo run forward.
        while (System.currentTimeMillis() < inTime)
        {
            servoButtonR.setDirection(Servo.Direction.FORWARD);
            servoButtonR.setPosition(0); //Keep in mind that this is a vex motor, so this command is equivalent to setting a motors power to 1.
        }
        servoButtonR.setPosition(.5);
    }

        //This function moves our right side button pusher out to around 1" from the front of the robot.
    public void rightButtonOut()
    {
        //Set the amount of time we need the servo to run out for to the current time + PUSHER_RUN_TIME constant.
        long outTime = System.currentTimeMillis() + PUSHER_RUN_TIME;

        //While the timer hasn't reached outTime, have the servo run forward.
        while (System.currentTimeMillis() < outTime)
        {
            servoButtonR.setDirection(Servo.Direction.REVERSE);
            servoButtonR.setPosition(0); //Keep in mind that this is a vex motor, so this command is equivalent to setting a motors power to 1.
        }
        servoButtonR.setPosition(.5);
    }


        //This function moves our right side button pusher out to around 1" from the front of the robot, then bring it back in.
    public void leftButtonOut()
    {
        //Set the amount of time we need the servo to run out for to the current time + PUSHER_RUN_TIME constant.
        long outTime = System.currentTimeMillis() + PUSHER_RUN_TIME;

        //While the timer hasn't reached outTime, have the servo run forward.
        while (System.currentTimeMillis() < outTime)
        {
            servoButtonL.setDirection(Servo.Direction.FORWARD);
            servoButtonL.setPosition(0); //Keep in mind that this is a vex motor, so this command is equivalent to setting a motors power to 1.
        }
        servoButtonL.setPosition(.5);
    }

    public void leftButtonIn()
    {
        //Set the amount of time we need the servo to run in for to the current time + PUSHER_RUN_TIME constant.
        long inTime = System.currentTimeMillis() + PUSHER_RUN_TIME;

        //While the timer hasn't reached inTime, have the servo run forward.
        while (System.currentTimeMillis() < inTime)
        {
            servoButtonL.setDirection(Servo.Direction.REVERSE);
            servoButtonL.setPosition(0);
        }
        servoButtonL.setPosition(.5);
    }

    public void loadBall()
    {
        //Set the amount of time we need the servo to run in for to the current time + PUSHER_RUN_TIME constant.
        long timeCycle = System.currentTimeMillis() + CYCLE_TIME;

        //While the timer hasn't reached inTime, have the servo run forward.
        while (System.currentTimeMillis() < timeCycle)
        {
            servoLoad.setPower(1);
        }
        servoLoad.setPower(0);
    }




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

    int cmdMoveA(float distIn, float encoderCntPerIn, double power, DcMotor motor)
    {
        // Solve for encoder count target. (int) needed to cast result as integer
        int target = (int) (distIn * encoderCntPerIn);

        // Set motor target and power
        motor.setTargetPosition(target);
        motor.setPower(power);

        return target;
    }

    //This function contains the parameters for the three possibilities that the color sensor can detect.
    // As of current, this is simply used for driver feedback.
    char seeBeacon(int redVal, int blueVal, float hueVal)
    {
        {
            if (redVal > 2 && hueVal <= 0)
            {
                return 'R';
            }
            else if (blueVal > 2 && hueVal > 200)
            {
                return 'B';
            }
            else
            {
                return 'N';
            }
        }
    }

        //This function has the parameters to tell whether or not the color sensor can see the white line.
        //This is currently just being used for driver feedback.
    boolean seeLine (int redVal, int blueVal, int greenVal, float hueVal, int clearVal)
    {
        if (redVal <= 30 && redVal >= 40 && blueVal <= 30 && blueVal >= 40 && greenVal <= 30 && greenVal >= 40 && hueVal <= 90 && hueVal >= 120 && clearVal <= 40 && clearVal >= 50)
        {
            return true;
        }
        else
        {
            return false;
        }
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

    double scaleInput(double dVal)
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

}
