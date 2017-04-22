package org.firstinspires.ftc.teamcode.Teleop;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by andrew.keenan on 4/7/2017.
 */
@TeleOp(name="Worlds Teleop", group="Worlds")
public class TeleopWorlds extends LinearOpMode {
    Hardware3415 Artemis = new Hardware3415();
    public static double x, y, z, trueX, trueY;
    public static double frPower, flPower, brPower, blPower;
    public static final double FLAP_SPEED = 0.01 ;
    public static double flapPosition;

    public void runOpMode() {
        Artemis.init(hardwareMap, false);

        flapPosition = Artemis.flap.getPosition();

        // Send telemetry message to signify Balin waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        Artemis.navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get(Artemis.cdim),
                Artemis.NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                Artemis.NAVX_DEVICE_UPDATE_RATE_HZ);
        //Prevents Balin from running before callibration is complete
        while (Artemis.navx_device.isCalibrating() && !isStopRequested()) {
            telemetry.addData("Ready?", "No");
            telemetry.update();
        }
        telemetry.addData("Ready?", "Yes");
        telemetry.update();
        Artemis.navx_device.zeroYaw();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Balin.rollerRelease.setPosition(Balin.ROLLER_RELEASE_OUT);
        Artemis.flap.setPosition(Artemis.FLAP_UP);

        flapPosition = Artemis.flap.getPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            //Balin.limitState = Balin.limit.getState();

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                Artemis.navx_device.zeroYaw();
            }

            //Sets controls for linear slides on forklift
            if(Math.abs(gamepad2.right_stick_y)>.15){
                Artemis.lift(gamepad2.right_stick_y);
            } else{
                Artemis.lift(0);
            }
            if(gamepad2.a){
                Artemis.clampLeft.setPosition(Artemis.LEFT_CLAMP_CLAMP);
                Artemis.clampRight.setPosition(Artemis.RIGHT_CLAMP_CLAMP);
            }
            if(gamepad2.y){
                Artemis.clampLeft.setPosition(Artemis.LEFT_CLAMP_UP);
                Artemis.clampRight.setPosition(Artemis.RIGHT_CLAMP_UP);
            }

            //Sets controls for shooter
            if(Math.abs(gamepad2.left_stick_y)> .15) {
                Artemis.shoot(Math.abs(gamepad2.left_stick_y)*.85);
                telemetry.addData("power shooter", gamepad1.left_stick_y);
                telemetry.update();
            }

            else
                Artemis.shoot(0);

            //Sets control for turret
            if (gamepad2.right_trigger > .15) {
                Artemis.rotate.setPower(.86);
            } else if(gamepad2.left_trigger>.15) {
                Artemis.rotate.setPower(-.86);
            }

            //Sets controls for collector
            if (gamepad1.left_trigger > 0.15) {
                Artemis.collector.setPower(0.99);
            } else if (gamepad1.left_bumper) {
                Artemis.collector.setPower(-0.99);
            } else {
                Artemis.collector.setPower(0);
            }

            //Sets the gamepad values to x, y, and z
            z = gamepad1.right_stick_x; //sideways
            y = gamepad1.left_stick_y; //forward and backward
            x = gamepad1.left_stick_x; //rotation

            //Converts x and y to a different value based on the gyro value
            trueX = ((Math.cos(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * x) - ((Math.sin(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * y); //sets trueX to rotated value
            trueY = ((Math.sin(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * x) + ((Math.cos(Math.toRadians(360 - Artemis.convertYaw(Artemis.navx_device.getYaw())))) * y);

            //Sets trueX and trueY to its respective value
            x = trueX;
            y = trueY;

            //Sets the motor powers of the wheels to the correct power based on all three of the above gyro values and
            //scales them accordingly
            flPower = Range.scale((-x + y - z), -1, 1, -Artemis.MAX_MOTOR_SPEED, Artemis.MAX_MOTOR_SPEED);
            frPower = Range.scale((-x - y - z), -1, 1, -Artemis.MAX_MOTOR_SPEED, Artemis.MAX_MOTOR_SPEED);
            blPower = Range.scale((x + y - z), -1, 1, -Artemis.MAX_MOTOR_SPEED, Artemis.MAX_MOTOR_SPEED);
            brPower = Range.scale((x - y - z), -1, 1, -Artemis.MAX_MOTOR_SPEED, Artemis.MAX_MOTOR_SPEED);

            //Sets each motor power to the correct power
            Artemis.fl.setPower(flPower);
            Artemis.fr.setPower(frPower);
            Artemis.bl.setPower(blPower);
            Artemis.br.setPower(brPower);

            //Backup movement if the above method fails
            if (x == 0 && y == 0 && z == 0) {
                if (gamepad1.dpad_right) {
                    Artemis.bl.setPower(Artemis.MAX_MOTOR_SPEED);
                    Artemis.fl.setPower(Artemis.MAX_MOTOR_SPEED);
                } else if (gamepad1.dpad_up) {
                    Artemis.bl.setPower(-Artemis.MAX_MOTOR_SPEED);
                    Artemis.fl.setPower(-Artemis.MAX_MOTOR_SPEED);
                } else if (gamepad1.dpad_down) {
                    Artemis.br.setPower(Artemis.MAX_MOTOR_SPEED);
                    Artemis.fr.setPower(Artemis.MAX_MOTOR_SPEED);
                } else if (gamepad1.dpad_left) {
                    Artemis.br.setPower(-Artemis.MAX_MOTOR_SPEED);
                    Artemis.fr.setPower(-Artemis.MAX_MOTOR_SPEED);
                }
            }

            //Control servo toggling for beacon pushers and door
            Artemis.beaconPushLeftToggleReturnArray = Artemis.servoToggle(gamepad1.left_trigger > .15, Artemis.beaconPushLeft, Artemis.beaconPushLeftPositions, Artemis.beaconPushLeftPos, Artemis.beaconPushLeftButtonPressed, this);
            Artemis.beaconPushLeftPos = Artemis.beaconPushLeftToggleReturnArray[0];
            Artemis.beaconPushLeftButtonPressed = Artemis.beaconPushLeftToggleReturnArray[1] == 1;

            Artemis.beaconPushRightToggleReturnArray = Artemis.servoToggle(gamepad1.right_trigger > .15, Artemis.beaconPushRight, Artemis.beaconPushRightPositions, Artemis.beaconPushRightPos, Artemis.beaconPushRightButtonPressed, this);
            Artemis.beaconPushRightPos = Artemis.beaconPushRightToggleReturnArray[0];
            Artemis.beaconPushRightButtonPressed = Artemis.beaconPushRightToggleReturnArray[1] == 1;

            //Artemis.doorToggleReturnArray = Artemis.servoToggle(gamepad1.a, Artemis.door, Artemis.doorPositions, Artemis.doorPos, Artemis.doorButtonPressed, this);
            //Artemis.doorPos = Artemis.doorToggleReturnArray[0];
            //Artemis.doorButtonPressed = Artemis.doorToggleReturnArray[1] == 1;


            if(gamepad2.dpad_up) {
                Artemis.flap.setPosition(Artemis.FLAP_UP_POS);
            }
            else if(gamepad2.dpad_down) {
                Artemis.flap.setPosition(Artemis.FLAP_DOWN_POS);
            }
            else if(gamepad2.dpad_left || gamepad2.dpad_right) {
                Artemis.flap.setPosition(Artemis.FLAP_MID_POS);
            }


            //Returns important data to the driver.
            telemetry.addData("GamePad 1 Right Stick X Actual", gamepad1.right_stick_x);
            telemetry.addData("GamePad 1 Left Stick Y Actual", gamepad1.left_stick_y);
            telemetry.addData("GamePad 1 Left Stick X Actual", gamepad1.left_stick_x);
            telemetry.addData("GamePad 1 X", gamepad1.x);
            telemetry.addData("FR Power", Artemis.fr.getPower());
            telemetry.addData("FL Power", Artemis.fl.getPower());
            telemetry.addData("BR Power", Artemis.br.getPower());
            telemetry.addData("BL Power", Artemis.bl.getPower());
            telemetry.addData("Yaw", (Artemis.navx_device.getYaw()));
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            Artemis.waitForTick(40);
        }
    }
}
