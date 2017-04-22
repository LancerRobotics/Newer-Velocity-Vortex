package org.firstinspires.ftc.teamcode.Autonomous;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by Paspuleti on 3/14/2017.
 */
@Autonomous(name = "2 Beacon Red Shoot", group = "Championship")
public class B2Shoot2Red extends LinearOpMode{
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        balin.init(hardwareMap, true);
        balin.navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get(balin.cdim),
                balin.NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                balin.NAVX_DEVICE_UPDATE_RATE_HZ);
        while (balin.navx_device.isCalibrating() && !isStopRequested()) {
            telemetry.addData("Ready?", "No");
            telemetry.update();
        }
        telemetry.addData("Ready?", "Yes");
        telemetry.update();
        balin.changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        balin.navx_device.zeroYaw();
        balin.moveStraightnew(6, this); //Set this further back to allow for more error when turning the anglez
        AlignToWithinOf(20, 5, .25);
        balin.setDrivePower(0);
        /*
        balin.shoot(1.0);
        sleep(600);
        balin.shoot(0);

        //Get second particle into the shooter
        balin.door.setPosition(balin.DOOR_OPEN);
        balin.collector.setPower(1.0);
        sleep(2000);
        balin.collector.setPower(0);
        balin.door.setPosition(balin.DOOR_CLOSED);
        sleep(500);
        //Shoot second particle
        balin.shoot(1.0);
        sleep(600);
        balin.shoot(0);
        */
        sleep(500);


        //Turning for the white line
        AlignToWithinOf(-17.5, 2, .24); //Used Trig to calculate it might need to be adjusted but one degree of variance should be fine for detecting the line
        //First White Line code:
        sleep(500);
        boolean white_line = false;

        int iPos = balin.fr.getCurrentPosition();
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (balin.ods.getRawLightDetected() >= .5) {
                white_line = true;
            }
            balin.setDrivePower(balin.coast(64, balin.fr.getCurrentPosition(), iPos));
            telemetry.addData("Following White Line: ", balin.ods.getLightDetected());
            telemetry.addData("FL: ", balin.fl.getPower());
            telemetry.addData("FR: ", balin.fr.getPower());
            telemetry.addData("BR: ", balin.br.getPower());
            telemetry.addData("BL:", balin.bl.getPower());
            telemetry.update();
        }
        balin.setDrivePower(.2);
        sleep(200);
        balin.setDrivePower(0);
        sleep(300);
        AlignToWithinOf(-90, 2, .24);
        sleep(200);
        balin.rest();
        sleep(200);


        white_line = false;
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (balin.ods.getRawLightDetected() >= .5) {
                white_line = true;
            }
            balin.fr.setPower(.4);
            balin.br.setPower(-.4);
            balin.fl.setPower(-.4);
            balin.bl.setPower(.4);
        }
        balin.setDrivePower(0);
        sleep(200);
        AlignToWithinOf(-90, 2, .25);
        balin.adjustToDistanceShlok(16, .145, this);
        //Beacon Code
        telemetry.addLine("Detecting Color");
        telemetry.update();
        String color = balin.detectColor();
        balin.setDrivePower(0);
        if (color.equals("Red")) {
            balin.beaconPushLeft.setPosition(balin.LEFT_BEACON_PUSH);
            balin.beaconPushRight.setPosition(balin.RIGHT_BEACON_INITIAL_STATE);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(500);
        } else if (color.equals("Blue")) {
            balin.beaconPushLeft.setPosition(balin.LEFT_BEACON_INITIAL_STATE);
            balin.beaconPushRight.setPosition(balin.RIGHT_BEACON_PUSH);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(500);
        } else {
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(500);
        }
        telemetry.addLine("Hitting Beacon");
        telemetry.update();
        if (balin.beaconPushLeft.getPosition() == balin.LEFT_BEACON_INITIAL_STATE || balin.beaconPushRight.getPosition() == balin.RIGHT_BEACON_INITIAL_STATE) {
            balin.adjustToDistanceShlok(3, .2, this);
            telemetry.addLine("Hit Beacon");
            telemetry.update();
        }
        balin.rest();
        //balin.changeDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        balin.adjustToDistanceShlok(16, .145, this);
        sleep(100);
        white_line = false;
        iPos = balin.fr.getCurrentPosition();
        balin.fr.setPower(.7);
        balin.br.setPower(-.7);
        balin.fl.setPower(-.7);
        balin.bl.setPower(.72);
        telemetry.addLine("Starting to strafe");
        sleep(345);
        iPos = balin.fr.getCurrentPosition();
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (balin.ods.getRawLightDetected() >= .5) {
                white_line = true;
            }
            double power = balin.coastStrafe(18, balin.fr.getCurrentPosition(), iPos, this);
            //double power = .4;
            balin.fr.setPower(power);
            balin.br.setPower(-power);
            balin.fl.setPower(-power);
            balin.bl.setPower(power);
            telemetry.addData("Power: ", power);
            telemetry.update();

        }
        balin.setDrivePower(0);
        sleep(200);
        white_line = false;
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (balin.ods.getRawLightDetected() >= .5) {
                white_line = true;
            }
            balin.fr.setPower(-.4);
            balin.br.setPower(.4);
            balin.fl.setPower(.4);
            balin.bl.setPower(-.4);
        }
        AlignToWithinOf(-90, 2, .25);
        telemetry.addLine("Detecting Color");
        telemetry.update();
        color = balin.detectColor();
        balin.setDrivePower(0);
        if (color.equals("Red")) {
            balin.beaconPushLeft.setPosition(balin.LEFT_BEACON_PUSH);
            balin.beaconPushRight.setPosition(balin.RIGHT_BEACON_INITIAL_STATE);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(500);
        } else if (color.equals("Blue")) {
            balin.beaconPushLeft.setPosition(balin.LEFT_BEACON_INITIAL_STATE);
            balin.beaconPushRight.setPosition(balin.RIGHT_BEACON_PUSH);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(500);
        } else {
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(500);
        }
        telemetry.addLine("Hitting Beacon");
        telemetry.update();
        if (balin.beaconPushLeft.getPosition() == balin.LEFT_BEACON_INITIAL_STATE || balin.beaconPushRight.getPosition() == balin.RIGHT_BEACON_INITIAL_STATE) {
            balin.adjustToDistanceShlok(3, .2, this);
            telemetry.addLine("Hit Beacon");
            telemetry.update();
        }
        balin.rest();
        balin.setDrivePower(-.3);
        sleep(500);
        balin.setDrivePower(0);

    }
    // This is our most used, and most useful method.
    // We use it to turn to within our  threshold angle of 0.
    // The default power is .05 as that power performed with the best accuracy in our testing.
    public void AlignToWithin(double threshold){
        AlignToWithin(threshold, .05);
    }

    // The align to within algorithm uses our team's unique method of non-recovery turning.
    // Because the robot exits the control loop once the reading is past the target,
    // we can apply this logic and use it to control our turning to a precise degree of accuracy.
    public void AlignToWithin(double threshold, double power){
        TurnRightAbsolute(- threshold, power, this);
        TurnLeftAbsolute(threshold, power, this);
        TurnRightAbsolute(- threshold, power, this);
    }

    // The AlignToWithinOf method aligns to within threshold degrees of expected.
    // To do this, we subtract or add threshold to the expected reading,
    // following a similar structure to that of AlignToWithin
    public void AlignToWithinOf(double expected, double threshold, double power){
        TurnRightAbsolute(expected - threshold, power, this);
        TurnLeftAbsolute(expected + threshold, power, this);
        TurnRightAbsolute(expected - threshold, power, this);
        balin.changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void TurnLeftPLoop(double degrees, double maxPower, LinearOpMode opMode){
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is reccomended.


        if(!opMode.opModeIsActive())
            Finish(this);
        double power;
        while(balin.navx_device.getYaw() >= degrees && opMode.opModeIsActive()){
            power = Range.clip((balin.navx_device.getYaw() - degrees)/degrees, .05, maxPower);
            balin.br.setPower(power);
            balin.fr.setPower(power);
            balin.bl.setPower(-power);
            balin.fl.setPower(-power);
        }
        balin.setDrivePower(.3);
    }

    // Turns left to a precise angle, regardless of current lineup.
    public void TurnLeftAbsolute(double degrees, double power, LinearOpMode opMode){

        if(!opMode.opModeIsActive())
            Finish(this);
        while(balin.navx_device.getYaw() >= degrees && opMode.opModeIsActive()){
            balin.br.setPower(power);
            balin.fr.setPower(power);
            balin.bl.setPower(-power);
            balin.fl.setPower(-power);
        }
        balin.setDrivePower(0);

    }

    // Turns left to an angle based on the current reading.
    // This is most useful when we are using our file reading code,
    // or when we are later into the route and are worried the navX
    // may be drifted.
    public void TurnLeftRelative(double degrees, double power, LinearOpMode opMode){
        if(!balin.navx_device.isConnected()){
            // TurnLeftEnc(degrees, .10);
            return;
        }
        if(!opMode.opModeIsActive())
            Finish(this);
        double yaw = balin.navx_device.getYaw();
        degrees = -Math.abs(degrees);
        while(Math.abs(yaw - balin.navx_device.getYaw()) < degrees && opMode.opModeIsActive()){
            balin.br.setPower(power);
            balin.fr.setPower(power);
            balin.bl.setPower(-power);
            balin.fl.setPower(-power);
        }
        balin.setDrivePower(0);

    }

    // Functions just like TurnLeftAbsolute,
    // but allows us to input a positive number,
    // thus improving code legibility.
    public void TurnLeft(double degrees, double power){
        TurnLeftAbsolute(- Math.abs(degrees), power, this);
    }
    public void TurnRightPLoop(double degrees, double maxPower, LinearOpMode opMode){
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is reccomended.

        if(!opMode.opModeIsActive())
            Finish(this);
        double power;
        while(balin.navx_device.getYaw() <= degrees && opMode.opModeIsActive()){
            power = Range.clip((degrees - balin.navx_device.getYaw())/degrees, .05, maxPower);
            balin.br.setPower(power);
            balin.fr.setPower(power);
            balin.bl.setPower(-power);
            balin.fl.setPower(-power);
        }
        balin.setDrivePower(0);
    }
    public void TurnRightAbsolute(double degrees, double power, LinearOpMode opMode){
        if(!opMode.opModeIsActive())
            Finish(this);
        while(balin.navx_device.getYaw() <= degrees && opMode.opModeIsActive()){
            balin.br.setPower(-power);
            balin.fr.setPower(-power);
            balin.bl.setPower(power);
            balin.fl.setPower(power);
        }
        balin.setDrivePower(0);
    }
    public void TurnRight(double degrees, double power){
        TurnRightAbsolute(degrees, power, this);
    }
    public void TurnRightRelative(double degrees, double power, LinearOpMode opMode){

        if(!opMode.opModeIsActive())
            Finish(this);
        double yaw = balin.navx_device.getYaw();
        while(Math.abs(yaw - balin.navx_device.getYaw()) < degrees && opMode.opModeIsActive()){
            balin.br.setPower(-power);
            balin.fr.setPower(-power);
            balin.bl.setPower(power);
            balin.fl.setPower(power);
        }
        balin.setDrivePower(0);
    }
    public void Finish(LinearOpMode opMode){
        balin.navx_device.close();
        balin.colorSensor.close();
        balin.ods.close();
        opMode.stop();
    }
}
