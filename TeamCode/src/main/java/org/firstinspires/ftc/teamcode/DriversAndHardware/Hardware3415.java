package org.firstinspires.ftc.teamcode.DriversAndHardware;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Hardware3415 {
    /* Public OpMode members. */
    public DcMotor bl = null;
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor br = null;
    public DcMotor collector = null;
    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;
    public DcMotor piston = null;
    public Servo beaconPushLeft = null, beaconPushRight = null, clampLeft = null,
            clampRight = null, rollerRelease = null, door = null,  flap = null;
    public CRServo rotate = null;
    public AHRS navx_device = null;
    public ColorSensor colorSensor = null;
    //public ColorSensor lineTrackerF = null;
    //public ColorSensor lineTrackerB = null;
    //
    public OpticalDistanceSensor ods = null;
    public OpticalDistanceSensor odsB = null;
    //public OpticalDistanceSensor odsF = null;
    public boolean beaconBlue;

    public static final double LEFT_BEACON_INITIAL_STATE = 234.0 / 255;
    public static final double LEFT_BEACON_PUSH = 13.0 / 255;
    public static final double RIGHT_BEACON_PUSH= 230.0 / 255;
    public static final double RIGHT_BEACON_INITIAL_STATE = 0;
    public static final double LEFT_CLAMP_INITIAL_STATE = 9.0/255;
    public static final double LEFT_CLAMP_UP = 235.0/255;
    public static final double LEFT_CLAMP_CLAMP = 112.0 / 255;
    public static final double RIGHT_CLAMP_INITIAL_STATE = 244.0/255;
    public static final double RIGHT_CLAMP_UP = 0;
    public static final double RIGHT_CLAMP_CLAMP = 123.0 / 255;
    public static final double ROLLER_RELEASE_IN = 245.0 / 255;
    public static final double ROLLER_RELEASE_OUT = 0.0;
    public static final double DOOR_CLOSED = 230.0 / 255; //FIND VALUES FOR THIS
    public static final double DOOR_OPEN = 0.0;
    public static final double FLAP_DOWN = 1;
    public static final double FLAP_UP = 0;
    public static final double FLAP_UP_POS = FLAP_UP;
    public static final double FLAP_DOWN_POS = FLAP_UP_POS - 100.0/255;
    public static final double FLAP_MID_POS = FLAP_UP_POS - 50.0/255;


    //Motor, Servo, and Sensor Names
    public static final String servo = "servo";
    public static final String beaconPushLeftName = "beacon_left"; //Port Five
    public static final String beaconPushRightName = "beacon_right"; //Port Six
    public static final String clampLeftName = "clamp_left"; //Port Two
    public static final String clampRightName = "clamp_right"; //Port One
    public static final String rollerReleaseName = "roller_release";
    public static final String doorName = "door";
    public static final String flapperName = "flap"; //Port Three
    public static final String turretName = "turret"; //Port Four
    public static final String frName = "front_right";
    public static final String flName = "front_left";
    public static final String brName = "back_right";
    public static final String blName = "back_left";
    public static final String liftLeftName = "lift_left";
    public static final String liftRightName = "lift_right";
    public static final String pistonName = "piston";
    public static final String collectorName = "collector";
    public static final String colorSensorName = "color";
    //public static final String lineTrackerFName = "lineTrackerF";
    //public static final String lineTrackerBName = "lineTrackerB";
    public static final String odsName = "ods";
    public static final String odsbName = "odsB";
    public static final String sonarName = "sonar";
    public static final String sonarName2 = "sonar2";

    /* Other Important Data */
    public static final int NAVX_DIM_I2C_PORT = 0;
    public static final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    public static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFF = 0.15;
    public static final double MAX_MOTOR_SPEED = 0.75;
    public static final double WHEEL_DIAMETER = 3.93701;
    public static final String cdim = "dim";


    byte[] range2Cache;

    public static final int RANGE2_REG_START = 0x04;
    public static final int RANGE2_READ_LENGTH = 2;

    public I2cDevice RANGE2 = null;
    public I2cDeviceSynch RANGE2Reader = null;
    public DigitalChannel limit = null;
    public boolean limitState;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /*Toggle Stuff*/
    public static boolean beaconPushLeftButtonPressed = false;
    public static double[] beaconPushLeftPositions = {LEFT_BEACON_INITIAL_STATE, LEFT_BEACON_PUSH};
    public static int beaconPushLeftPos;
    public static int beaconPushLeftToggleReturnArray[] = new int[2];
    public static boolean beaconPushRightButtonPressed = false;
    public static double[] beaconPushRightPositions = {RIGHT_BEACON_INITIAL_STATE, RIGHT_BEACON_PUSH};
    public static int beaconPushRightPos;
    public static int beaconPushRightToggleReturnArray[] = new int[2];
    public static boolean doorButtonPressed = false;
    public static double[] doorPositions = {DOOR_CLOSED, DOOR_OPEN};
    public static int doorPos;
    public static int doorToggleReturnArray[] = new int[2];

    /* Constructor */
    public Hardware3415() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean autonomous) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fl = hwMap.dcMotor.get(flName);
        fr = hwMap.dcMotor.get(frName);
        bl = hwMap.dcMotor.get(blName);
        br = hwMap.dcMotor.get(brName);
        collector = hwMap.dcMotor.get(collectorName);
        piston = hwMap.dcMotor.get(pistonName);
        liftRight = hwMap.dcMotor.get(liftRightName);
        liftLeft = hwMap.dcMotor.get(liftLeftName);
        if (autonomous) {
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
        }
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);


        piston.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        restAllMotors();
        piston.setPower(0);
        collector.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        piston.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        beaconPushLeft = hwMap.servo.get(beaconPushLeftName);
        beaconPushRight = hwMap.servo.get(beaconPushRightName);
        clampLeft = hwMap.servo.get(clampLeftName);
        clampRight = hwMap.servo.get(clampRightName);
        rotate = hwMap.crservo.get(turretName);
        flap = hwMap.servo.get(flapperName);
        rotate.setPower(0);
        flap.setPosition(FLAP_DOWN);
        if (autonomous) {
            beaconPushLeft.setPosition(LEFT_BEACON_PUSH);
            beaconPushRight.setPosition(RIGHT_BEACON_PUSH);
            clampLeft.setPosition(LEFT_CLAMP_INITIAL_STATE);
            clampRight.setPosition(RIGHT_CLAMP_INITIAL_STATE);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Define all sensors

            //lineTrackerF = hwMap.colorSensor.get(lineTrackerFName);
            //lineTrackerB = hwMap.colorSensor.get(lineTrackerBName);
            ods = hwMap.opticalDistanceSensor.get(odsName);

            RANGE2 = hwMap.i2cDevice.get(sonarName2);
            RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, I2cAddr.create8bit(0x38), false);
            RANGE2Reader.engage();

            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            changeDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {

            beaconPushLeftPos = 1;
            beaconPushLeft.setPosition(beaconPushLeftPositions[0]);
            beaconPushRightPos = 1;
            beaconPushRight.setPosition(beaconPushRightPositions[0]);
            clampLeft.setPosition(LEFT_CLAMP_INITIAL_STATE);
            clampRight.setPosition(RIGHT_CLAMP_INITIAL_STATE);
        }
    }


    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public static float convertYaw(double yaw) {
        if (yaw <= 0) {
            yaw = 360 + yaw; //if yaw is negative, make it positive (makes the turn easier to visualize)
        }
        return (float) yaw;
    }

    public void restAllMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        collector.setPower(0);
        liftLeft.setPower(0);
        liftRight.setPower(0);
        piston.setPower(0);
    }

    public int[] servoToggle(boolean button, Servo servo, double[] positions, int currentPos, boolean pressed, LinearOpMode opMode) {
        //Creates a variable saying the number of servo positions
        int servoPositions = positions.length;

        //Checks to see if a button is pressed
        if (button && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            pressed = true;
        }

        //If the button is pressed, the servo is set to the value following the previous servo value in the values array.
        //The method also t ells us what is the current position (1, 2, or 3) of the servo and will say if the button is no longer pressed
        if (pressed && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (servoPositions == 2 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                if (currentPos == 1 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    servo.setPosition(positions[1]);
                    if (!button && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                        pressed = false;
                        currentPos = 2;
                    }
                } else if (currentPos == 2 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    servo.setPosition(positions[0]);
                    if (!button && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                        pressed = false;
                        currentPos = 1;
                    }
                }
            } else if (servoPositions == 3 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                if (currentPos == 1 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    servo.setPosition(positions[1]);
                    if (!button && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                        pressed = false;
                        currentPos = 2;
                    }
                } else if (currentPos == 2 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    servo.setPosition(positions[2]);
                    if (!button && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                        pressed = false;
                        currentPos = 3;
                    }
                } else if (currentPos == 3 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    servo.setPosition(positions[0]);
                    if (!button && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                        pressed = false;
                        currentPos = 1;
                    }
                }
            }
        }

        //Returns values for toggle return arrays
        int boolPressed = 0;
        if (pressed && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            boolPressed = 1;
        }
        int returnArray[] = new int[2];
        returnArray[0] = currentPos;
        returnArray[1] = boolPressed;
        return returnArray;
    }

    public void lift(double power) {
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    //Method to run piston motor at the same power
    public void shoot(double power) {
        piston.setPower(power);
    }

    public void changeDriveMode(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    public void setDriveTarget(int targetTick) {
        br.setTargetPosition(targetTick);
        bl.setTargetPosition(targetTick);
        fl.setTargetPosition(targetTick);
        fr.setTargetPosition(targetTick);
    }

    public void setDrivePower(double power) {
        fr.setPower(power);
        fl.setPower(power);
        br.setPower(power);
        bl.setPower(power);

    }

    public boolean motorsBusy(LinearOpMode opMode){
        if(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy() && opMode.opModeIsActive() && !opMode.isStopRequested())
            return true;
        return false;
    }

    public boolean motorsReset(LinearOpMode opMode) {
        if (fr.getCurrentPosition() == 0 && bl.getCurrentPosition() == 0 && fl.getCurrentPosition() == 0 && br.getCurrentPosition() == 0 && opMode.opModeIsActive() && !opMode.isStopRequested())
            return true;
        return false;
    }

    public boolean motorsTarget(int targetTick, LinearOpMode opMode) {
        if ((fl.getCurrentPosition() < (targetTick - 50) || bl.getCurrentPosition() < (targetTick - 50) || br.getCurrentPosition() < (targetTick - 50) || fr.getCurrentPosition() < (targetTick - 50)) && opMode.opModeIsActive() && !opMode.isStopRequested())
            return true;
        return false;
    }
    public void Move(double centimeters, double power) {
        changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double ticks = (int) (centimeters * 1140.0 / (4.0 * Math.PI * 2.0));
        int RBPos = Math.abs(br.getCurrentPosition());
        int RFPos = Math.abs(fr.getCurrentPosition());
        int LBPos = Math.abs(bl.getCurrentPosition());
        int LFPos = Math.abs(fl.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        while(avg < ticks) {
            setDrivePower(power);
            RBPos = Math.abs(br.getCurrentPosition());
            RFPos = Math.abs(fr.getCurrentPosition());
            LBPos = Math.abs(bl.getCurrentPosition());
            LFPos = Math.abs(fl.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        setDrivePower(0);
    }

    public boolean moveStraightnew(double inches, LinearOpMode opMode){
        while(br.getCurrentPosition()!=0 && opMode.opModeIsActive()) {
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        int targetTick = (int) (inches * 1140.0 / (4.0 * Math.PI * 2.0));
        br.setTargetPosition(targetTick);
        if(inches >  0) {
            setDrivePower(.35);
            while (br.getCurrentPosition() < targetTick && br.isBusy() && opMode.opModeIsActive() && !opMode.isStopRequested()) {

            }
        }
        else{
            setDrivePower(-.35);
            while(br.getCurrentPosition()> targetTick && br.isBusy() && opMode.opModeIsActive() && !opMode.isStopRequested()) {

            }
        }

        setDrivePower(0);
        return true;
    }

    public boolean turnNew(double inches, LinearOpMode opMode){
        changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int targetTick = (int) (inches * 1140.0 / (4.0 * Math.PI * 2.0));
        fr.setTargetPosition(targetTick);
        fr.setPower(-.25);
        fl.setPower(.25);
        br.setPower(-.25);
        bl.setPower(.25);
        while(fr.getCurrentPosition()< targetTick && fr.isBusy() && opMode.opModeIsActive() && !opMode.isStopRequested()){

        }
        setDrivePower(0);
        return true;
    }
    public void moveStraight1(int inches, boolean backwards, LinearOpMode opMode) {
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetTick = (int) (inches * 1140.0 / (4.0 * Math.PI * 2.0));
        if (!backwards && opMode.opModeIsActive() && !opMode.isStopRequested()) {

            if (motorsReset(opMode)) {
                fr.setTargetPosition(targetTick);
                fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            while (!motorsTarget(targetTick, opMode) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                setDrivePower(.5);
                opMode.telemetry.addData("Encoders Reset?", motorsReset(opMode));
                opMode.telemetry.addData("Current tick values", fl.getCurrentPosition());
                opMode.telemetry.addData("Current tick values", br.getCurrentPosition());
                opMode.telemetry.update();
                waitForTick(40);
            }
            setDrivePower(0.0);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            if (motorsReset(opMode) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                setDriveTarget(targetTick);
                changeDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            while (!motorsTarget(targetTick, opMode) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                setDrivePower(.5);
                waitForTick(40);
            }
            rest();
            changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }


    public void moveStraight(int inches, boolean backwards, LinearOpMode opMode) {
        changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        changeDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetTick = (int) (inches * 1140.0 / (4.0 * Math.PI * 2.0));
        if (!backwards && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (motorsReset(opMode) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                setDriveTarget(targetTick);
                changeDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (!motorsTarget(targetTick, opMode) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    //setDrivePower(coast(targetTick, smallest(fl.getCurrentPosition(), bl.getCurrentPosition(), fr.getCurrentPosition(), br.getCurrentPosition())));
                    setDrivePower(.3);
                    opMode.telemetry.addData("Encoders Reset?", motorsReset(opMode));
                    opMode.telemetry.addData("Current tick values", fl.getCurrentPosition());
                    opMode.telemetry.addData("Current tick values", br.getCurrentPosition());
                    opMode.telemetry.update();
                    waitForTick(40);
                }
            }
            changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rest();
        } else {
            if (motorsReset(opMode) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                setDriveTarget(targetTick);
                changeDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                while ((fl.getCurrentPosition() < (fl.getTargetPosition() - 50) || bl.getCurrentPosition() < (bl.getTargetPosition() - 50) || br.getCurrentPosition() < (br.getTargetPosition() - 50) || fr.getCurrentPosition() < (fr.getTargetPosition() - 50)) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    //setDrivePower(-coast(targetTick, smallest(fl.getCurrentPosition(), bl.getCurrentPosition(), fr.getCurrentPosition(), br.getCurrentPosition())));
                    setDrivePower(-.3);
                    waitForTick(40);
                }
            }
            changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rest();
        }
    }


    public int smallest(int a, int b, int c, int d) {
        int smallest;
        if (Math.abs(a) > Math.abs(b)) {
            smallest = Math.abs(b);
        } else {
            smallest = Math.abs(a);
        }
        if (smallest > Math.abs(c)) {
            smallest = Math.abs(c);
        }
        if (smallest > Math.abs(d)) {
            smallest = Math.abs(d);
        }
        return smallest;
    }

    public static double coast(int target, int currentPosition, int iPos) {
        target = (int) (target * 1140.0 / (4.0 * Math.PI * 2.0));
        target = Math.abs(target);
        currentPosition = Math.abs(currentPosition);
        double power = ((currentPosition-iPos) * 1.0) / target;
        power = .9 / (1 + Math.pow(2.7182, 1.34 * power));
        return power;
    }
    public static double coastStrafe(int target, int currentPosition, int iPos, LinearOpMode opMode) {
        target = (int) (target * 1140.0 / (4.0 * Math.PI * 2.0));
        target = Math.abs(target);
        currentPosition = Math.abs(currentPosition);
        double power = ((currentPosition-iPos) * 1.0) / target;
        power = 1.4 / (1 + Math.pow(2.7182, .8 * power));
        opMode.telemetry.addData("power: ", power);
        return power;
    }

    public void timeMove(int seconds, boolean backwards, LinearOpMode opMode) {
        if (!backwards && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            fr.setPower(.4);
            fl.setPower(.4);
            br.setPower(.4);
            bl.setPower(.4);
            opMode.sleep(seconds * 1000);
        } else {
            if(opMode.opModeIsActive() && !opMode.isStopRequested()) {
                fr.setPower(-.4);
                fl.setPower(-.4);
                br.setPower(-.4);
                bl.setPower(-.4);
                opMode.sleep(seconds * 1000);
            }
        }
    }

    //Stops all motors on the drive train
    public void rest() {
        setDrivePower(0);

    }

    //Sets the DIRECTION the robot is going, based on the error, for gyro turn
    public double getSteer(double error, double speed) {
        int powerMultiplier = 1;
        if (error < 0) {
            powerMultiplier = -1;
        }
        return Range.clip(powerMultiplier * speed, -1, 1);
    }

    //Gives the DIFFERENCE between current and target angle->as robotError
    public double getError(double targetAngle, LinearOpMode opMode) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - navx_device.getYaw();
        while (robotError > 180 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            robotError -= 360;
        }
        while (robotError <= -180 && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            robotError += 360;
        }
        return robotError;
    }

    //Method that tells the motors the speeds they need to turn.
    public void turn(double power) {
        fr.setPower(-power);
        br.setPower(-power);
        fl.setPower(power);
        bl.setPower(power);
    }

    //Method that has the actual robot turn
    public boolean onHeading(double speed, double angle, double PCoeff, LinearOpMode opMode) {

        //Creates the variables needed for the method
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        //Determine turn power based on how far off the robot is from the correct angle
        error = getError(angle, opMode);

        //Tells the robot to move according to the angle of the robot
        if (Math.abs(error) <= HEADING_THRESHOLD && opMode.opModeIsActive() && !opMode.isStopRequested()) { //Allows the bot to reach an angle within the range of heading_threshold
            rest(); // stops all motors if navx returns a heading that is within
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else { // will try and get back to the desired angle if it is not within the range of desired angles
            steer = getSteer(error, speed); //calls the method to adjust the angle to the desired angle
            rightSpeed = steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        if (opMode.opModeIsActive() && !opMode.isStopRequested()) turn(rightSpeed);

        // Display information for the driver.
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        opMode.telemetry.addData("Yaw", navx_device.getYaw());
        return onTarget;
    }

    // Method that is called to turn the robot goes from -180 to 180 degrees
    public void gyroAngle(double angle, double speed, LinearOpMode opMode) {
        //Zero's the gyro value

        navx_device.zeroYaw();
        //Turns the robot
        if (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            // keep looping while we are still active, and not on heading.
            while (opMode.opModeIsActive() && !opMode.isStopRequested() && !onHeading(speed, angle, P_TURN_COEFF, opMode)) {
                // Update telemetry & Allow time for other processes to run.
                opMode.telemetry.addData("Current Yaw", navx_device.getYaw());
                opMode.telemetry.addData("Target Yaw", angle);
                opMode.telemetry.update();
                waitForTick(40);
            }
        }
        //Brakes all motors
        if (opMode.opModeIsActive() && !opMode.isStopRequested()) rest();

    }


    public void restAndSleep(LinearOpMode opMode) {
        if (opMode.opModeIsActive() && !opMode.isStopRequested()) rest();
        opMode.sleep(100);
        opMode.telemetry.update();
    }

    //Takes in the gyro values and converts to degrees from 0-360
    public float getYaw() {
        float yaw = convertYaw(navx_device.getYaw());
        return yaw;
    }
    public double getLight(LinearOpMode opMode){
        double odsReading = ods.getLightDetected();
        opMode.telemetry.addData("odsReading", "%5.2f", odsReading);
        opMode.telemetry.update();
        return odsReading;
    }

    public int[] getRGB() {
        int red = colorSensor.red(); // store the values the color sensor returns
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        int[] rgb = {red, green, blue};
        return rgb;
    }


    /* public int[] getRGBF(){
         int red = lineTrackerF.red();
         int blue = lineTrackerF.blue();
         int green = lineTrackerF.green();
         int[] rgb ={red, green, blue};
         return rgb;
     }*/
   /* public int[] getRGBB(){
        int red = lineTrackerB.red();
        int blue = lineTrackerB.blue();
        int green = lineTrackerB.green();
        int[] rgb = {red, green, blue};
        return rgb;
*/
    public boolean detectAColor(){
        int rgb[] = getRGB();
        if(rgb[0]> 1 && rgb[2]>1){
            return true;
        }
        return false;
    }
    public boolean BeaconColor()
    {
        int[] rgb = getRGB();
        boolean color = false;
        return color;
    }

    public double biggestDouble(double a, double b, double c, double d) {
        double biggest;
        if (Math.abs(a) < Math.abs(b)) {
            biggest = Math.abs(b);
        } else {
            biggest = Math.abs(a);
        }
        if (biggest < Math.abs(c)) {
            biggest = Math.abs(c);
        }
        if (biggest < Math.abs(d)) {
            biggest = Math.abs(d);
        }
        return biggest;
    }

    public void normalizeSpeedStrafe(LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        float flEncoderBefore = Math.abs(fl.getCurrentPosition());
        double flTimeBefore = timer.time();
        float frEncoderBefore = Math.abs(fr.getCurrentPosition());
        double frTimeBefore = timer.time();
        float blEncoderBefore = Math.abs(bl.getCurrentPosition());
        double blTimeBefore = timer.time();
        float brEncoderBefore = Math.abs(br.getCurrentPosition());
        double brTimeBefore = timer.time();
        opMode.sleep(200);
        float flEncoderAfter = Math.abs(fl.getCurrentPosition());
        double flTimeAfter = timer.time();
        float frEncoderAfter = Math.abs(fr.getCurrentPosition());
        double frTimeAfter = timer.time();
        float blEncoderAfter = Math.abs(bl.getCurrentPosition());
        double blTimeAfter = timer.time();
        float brEncoderAfter = Math.abs(br.getCurrentPosition());
        double brTimeAfter = timer.time();
        float flChangeInTick = Math.abs(flEncoderAfter - flEncoderBefore);
        float frChangeInTick = Math.abs(frEncoderAfter - frEncoderBefore);
        float blChangeInTick = Math.abs(blEncoderAfter - blEncoderBefore);
        float brChangeInTick = Math.abs(brEncoderAfter - brEncoderBefore);
        double flChangeInRad = (2.0 * Math.PI * flChangeInTick) / 560.0;
        double frChangeInRad = (2.0 * Math.PI * frChangeInTick) / 560.0;
        double blChangeInRad = (2.0 * Math.PI * blChangeInTick) / 560.0;
        double brChangeInRad = (2.0 * Math.PI * brChangeInTick) / 560.0;
        double flAngVel = flChangeInRad / (flTimeAfter - flTimeBefore);
        double frAngVel = frChangeInRad / (frTimeAfter - frTimeBefore);
        double blAngVel = blChangeInRad / (blTimeAfter - blTimeBefore);
        double brAngVel = brChangeInRad / (brTimeAfter - brTimeBefore);
        double flTranVel = flAngVel * (WHEEL_DIAMETER/2.0);
        double frTranVel = frAngVel * (WHEEL_DIAMETER/2.0);
        double blTranVel = blAngVel * (WHEEL_DIAMETER/2.0);
        double brTranVel = brAngVel * (WHEEL_DIAMETER/2.0);
        opMode.telemetry.addData("FL TRAN VEL", flTranVel);
        opMode.telemetry.addData("FR TRAN VEL", frTranVel);
        opMode.telemetry.addData("BL TRAN VEL", blTranVel);
        opMode.telemetry.addData("BR TRAN VEL", brTranVel);
        opMode.telemetry.update();
        if(!opMode.isStopRequested() && opMode.opModeIsActive() && (flTranVel != frTranVel || flTranVel != blTranVel || flTranVel != brTranVel)) {
            double biggestTranVel = biggestDouble(flTranVel, frTranVel, blTranVel, brTranVel), flAdjustedPower, frAdjustedPower, blAdjustedPower, brAdjustedPower;
            if(flTranVel != biggestTranVel) {
                flAdjustedPower = (fl.getPower() * biggestTranVel)/flTranVel;
            }
            else {
                flAdjustedPower = fl.getPower();
            }
            if(frTranVel != biggestTranVel) {
                frAdjustedPower = (fr.getPower() * biggestTranVel)/frTranVel;
            }
            else {
                frAdjustedPower = fr.getPower();
            }
            if(blTranVel != biggestTranVel) {
                blAdjustedPower = (bl.getPower() * biggestTranVel)/blTranVel;
            }
            else {
                blAdjustedPower = bl.getPower();
            }
            if(brTranVel != biggestTranVel) {
                brAdjustedPower = (br.getPower() * biggestTranVel)/brTranVel;
            }
            else {
                brAdjustedPower = br.getPower();
            }
            fl.setPower(Range.clip(flAdjustedPower,-1,1));
            fr.setPower(Range.clip(frAdjustedPower,-1,1));
            bl.setPower(Range.clip(blAdjustedPower,-1,1));
            br.setPower(Range.clip(brAdjustedPower,-1,1));
            opMode.telemetry.addData("FL Power", flAdjustedPower);
            opMode.telemetry.addData("FR Power", frAdjustedPower);
            opMode.telemetry.addData("BL Power", blAdjustedPower);
            opMode.telemetry.addData("BR Power", brAdjustedPower);
            opMode.telemetry.update();
        }
    }



    public double readSonar2() {
        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        double sonarData = range2Cache[0] & 0xFF;
        return sonarData;
    }

    public void adjustToDistance(double distance, double power, LinearOpMode opMode) {
        boolean done = false;
        while(opMode.opModeIsActive() && !opMode.isStopRequested() && !done) {
            if (readSonar2() < distance - 2.5) {
                while (readSonar2() < distance - 2.5 && opMode.opModeIsActive() && !opMode.isStopRequested() && readSonar2()<50) {
                    setDrivePower(-power);
                }
            } else if (readSonar2() > distance + 2.5) {
                while (readSonar2() > distance + 2.5 && opMode.opModeIsActive() && !opMode.isStopRequested() && readSonar2()<50) {
                    setDrivePower(power);
                }
            } else {
                done = true;
            }
            opMode.telemetry.addData("Range: ", readSonar2());
            restAndSleep(opMode);
            opMode.telemetry.update();
        }
    }
    public void adjustToDistanceShlok(double distance, double power, LinearOpMode opMode){
        while(readSonar2()>50){

        }
        double difference = readSonar2()-distance;
        moveStraightnew(difference*0.393701, opMode);
    }

    public void adjustToDistanceBad(double distance, double power, LinearOpMode opMode) {
        boolean done = false;
        while(opMode.opModeIsActive() && !opMode.isStopRequested() && !done) {
            if (readSonar2() < distance - 7.5) {
                while (readSonar2() < distance - 7.5 && opMode.opModeIsActive() && !opMode.isStopRequested() && readSonar2()<50) {
                    setDrivePower(-power);
                }
            } else if (readSonar2() > distance + 7.5) {
                while (readSonar2() > distance + 7.5 && opMode.opModeIsActive() && !opMode.isStopRequested() && readSonar2()<50) {
                    setDrivePower(power);
                }
            } else {
                done = true;
            }
            opMode.telemetry.addData("Range: ", readSonar2());
            restAndSleep(opMode);
            opMode.telemetry.update();
        }
    }

    public String detectColor(){
        if(colorSensor.red()>colorSensor.blue()){
            return "Red";
        }
        else if(colorSensor.blue()>colorSensor.red()){
            return "Blue";
        }
        else{
            return "N/A";
        }
    }
    public void Straighten(double a){

    }



}