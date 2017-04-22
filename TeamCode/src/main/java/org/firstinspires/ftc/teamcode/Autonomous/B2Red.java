package org.firstinspires.ftc.teamcode.Autonomous;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by Paspuleti on 4/20/2017.
 */
@Autonomous(name = "Red Auton", group = "Autonomous")
public class B2Red extends LinearOpMode{
    Hardware3415 Artemis = new Hardware3415();
    public void runOpMode(){
        Artemis.init(hardwareMap, true);
        Artemis.navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get(Artemis.cdim),
                Artemis.NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                Artemis.NAVX_DEVICE_UPDATE_RATE_HZ);
        while (Artemis.navx_device.isCalibrating() && !isStopRequested()) {
            telemetry.addData("Ready?", "No");
            telemetry.update();
        }
        telemetry.addData("Ready?", "Yes");
        telemetry.update();
        Artemis.changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        Artemis.navx_device.zeroYaw();
        /*Artemis.flap.setPosition(Artemis.FLAP_UP_POS);
        sleep(500);
        sleep(500);
        Artemis.shoot(.75);
        sleep(600);
        Artemis.shoot(0);

        //Get second particle into the shooter

        Artemis.collector.setPower(1.0);
        sleep(2000);
        Artemis.collector.setPower(0);

        sleep(500);
        //Shoot second particle
        Artemis.shoot(.75);
        sleep(600);
        Artemis.shoot(0);
        */
        Artemis.moveStraight(6, this); //Set this further back to allow for more error when turning the angle
        //Turning for the white line
        sleep(500);
        AlignToWithinOf(-21, 1, .29); //Used Trig to calculate it might need to be adjusted but one degree of variance should be fine for detecting the line
        //First White Line code:
        sleep(500);
        boolean white_line = false;
        int iPos = Artemis.fr.getCurrentPosition();
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (Artemis.ods.getRawLightDetected() >= 1) {
                white_line = true;
            }
            Artemis.setDrivePower(Artemis.coast(84, Artemis.br.getCurrentPosition(), iPos));
            telemetry.addData("Following White Line: ", Artemis.ods.getLightDetected());
            telemetry.addData("FL: ", Artemis.fl.getPower());
            telemetry.addData("FR: ", Artemis.fr.getPower());
            telemetry.addData("BR: ", Artemis.br.getPower());
            telemetry.addData("BL:", Artemis.bl.getPower());
            telemetry.addData("Current Position: ", Artemis.fr.getCurrentPosition());
            telemetry.update();
        }
        Artemis.setDrivePower(.2);
        sleep(200);
        Artemis.setDrivePower(0);
        sleep(200);
        AlignToWithinOf(-90, 2, .31);
        Artemis.rest();
        sleep(200);

        white_line = false;

        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (Artemis.ods.getRawLightDetected() >= 1) {
                white_line = true;
            }
            Artemis.fr.setPower(.43);
            Artemis.br.setPower(-.43);
            Artemis.fl.setPower(-.43);
            Artemis.bl.setPower(.43);
        }

        Artemis.setDrivePower(0);
        sleep(200);
        AlignToWithinOf(-90, 2, .31);
        Artemis.adjustToDistanceShlok(19, .145, this);
        //Beacon Code
        telemetry.addLine("Detecting Color");
        telemetry.update();
        String color = Artemis.detectColor();
        Artemis.setDrivePower(0);
        if (color.equals("Blue")) {
            Artemis.beaconPushLeft.setPosition(Artemis.LEFT_BEACON_PUSH);
            Artemis.beaconPushRight.setPosition(Artemis.RIGHT_BEACON_INITIAL_STATE);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(200);
        } else if (color.equals("Red")) {
            Artemis.beaconPushLeft.setPosition(Artemis.LEFT_BEACON_INITIAL_STATE);
            Artemis.beaconPushRight.setPosition(Artemis.RIGHT_BEACON_PUSH);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(200);
        } else {
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(200);
        }
        telemetry.addLine("Hitting Beacon");
        telemetry.update();
        if (Artemis.beaconPushLeft.getPosition() == Artemis.LEFT_BEACON_INITIAL_STATE || Artemis.beaconPushRight.getPosition() == Artemis.RIGHT_BEACON_INITIAL_STATE) {
            Artemis.setDrivePower(.3);
            sleep(700);
            Artemis.setDrivePower(0);
            telemetry.addLine("Hit Beacon");
            telemetry.update();
        }
        Artemis.rest();
        Artemis.adjustToDistanceShlok(19.5, .145, this);
        sleep(100);
        white_line = false;
        iPos = Artemis.br.getCurrentPosition();
        Artemis.fr.setPower(.7);
        Artemis.br.setPower(-.7);
        Artemis.fl.setPower(-.7);
        Artemis.bl.setPower(.7);
        telemetry.addLine("Starting to strafe");
        sleep(345);
        iPos = Artemis.fr.getCurrentPosition();
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (Artemis.ods.getRawLightDetected() >= 1) {
                white_line = true;
            }
            double power = Artemis.coastStrafe(80, Artemis.fr.getCurrentPosition(), iPos, this);
            //double power = .4;
            Artemis.fr.setPower(power);
            Artemis.br.setPower(-power);
            Artemis.fl.setPower(-power);
            Artemis.bl.setPower(power);
            telemetry.addData("Power: ", power);
            telemetry.addData("Current Position: ", Artemis.br.getCurrentPosition());
            telemetry.update();


        }
        Artemis.setDrivePower(0);
        sleep(200);
        white_line = false;
        while (!white_line && opModeIsActive() && !isStopRequested()) {
            if (Artemis.ods.getRawLightDetected() >= 1) {
                white_line = true;
            }
            Artemis.fr.setPower(-.43);
            Artemis.br.setPower(.43);
            Artemis.fl.setPower(.43);
            Artemis.bl.setPower(-.43);
        }
        AlignToWithinOf(-90, 2, .31);
        Artemis.adjustToDistanceShlok(19, .145, this);
        telemetry.addLine("Detecting Color");
        telemetry.update();
        color = Artemis.detectColor();
        Artemis.setDrivePower(0);
        if (color.equals("Blue")) {
            Artemis.beaconPushLeft.setPosition(Artemis.LEFT_BEACON_PUSH);
            Artemis.beaconPushRight.setPosition(Artemis.RIGHT_BEACON_INITIAL_STATE);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(200);
        } else if (color.equals("Red")) {
            Artemis.beaconPushLeft.setPosition(Artemis.LEFT_BEACON_INITIAL_STATE);
            Artemis.beaconPushRight.setPosition(Artemis.RIGHT_BEACON_PUSH);
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(200);
        } else {
            telemetry.addData("Color: ", color);
            telemetry.update();
            sleep(200);
        }
        telemetry.addLine("Hitting Beacon");
        telemetry.update();
        if (Artemis.beaconPushLeft.getPosition() == Artemis.LEFT_BEACON_INITIAL_STATE || Artemis.beaconPushRight.getPosition() == Artemis.RIGHT_BEACON_INITIAL_STATE) {
            Artemis.setDrivePower(.3);
            sleep(700);
            Artemis.setDrivePower(0);
            telemetry.addLine("Hit Beacon");
            telemetry.update();
        }
        Artemis.rest();
        Artemis.adjustToDistanceShlok(30, .2, this);
        AlignToWithinOf(-160, 5, .4);
        Artemis.fr.setPower(.5);
        Artemis.br.setPower(.5);
        Artemis.fl.setPower(.5);
        Artemis.bl.setPower(.5);
        sleep(1400);
        Artemis.setDrivePower(0);

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
        Artemis.changeDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void TurnLeftPLoop(double degrees, double maxPower, LinearOpMode opMode){
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is reccomended.


        if(!opMode.opModeIsActive())
            Finish(this);
        double power;
        while(Artemis.navx_device.getYaw() >= degrees && opMode.opModeIsActive()){
            power = Range.clip((Artemis.navx_device.getYaw() - degrees)/degrees, .05, maxPower);
            Artemis.br.setPower(power);
            Artemis.fr.setPower(power);
            Artemis.bl.setPower(-power);
            Artemis.fl.setPower(-power);
        }
        Artemis.setDrivePower(.3);
    }

    // Turns left to a precise angle, regardless of current lineup.
    public void TurnLeftAbsolute(double degrees, double power, LinearOpMode opMode){

        if(!opMode.opModeIsActive())
            Finish(this);
        while(Artemis.navx_device.getYaw() >= degrees && opMode.opModeIsActive()){
            Artemis.br.setPower(power);
            Artemis.fr.setPower(power);
            Artemis.bl.setPower(-power);
            Artemis.fl.setPower(-power);
        }
        Artemis.setDrivePower(0);

    }

    // Turns left to an angle based on the current reading.
    // This is most useful when we are using our file reading code,
    // or when we are later into the route and are worried the navX
    // may be drifted.
    public void TurnLeftRelative(double degrees, double power, LinearOpMode opMode){
        if(!Artemis.navx_device.isConnected()){
            // TurnLeftEnc(degrees, .10);
            return;
        }
        if(!opMode.opModeIsActive())
            Finish(this);
        double yaw = Artemis.navx_device.getYaw();
        degrees = -Math.abs(degrees);
        while(Math.abs(yaw - Artemis.navx_device.getYaw()) < degrees && opMode.opModeIsActive()){
            Artemis.br.setPower(power);
            Artemis.fr.setPower(power);
            Artemis.bl.setPower(-power);
            Artemis.fl.setPower(-power);
        }
        Artemis.setDrivePower(0);

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
        while(Artemis.navx_device.getYaw() <= degrees && opMode.opModeIsActive()){
            power = Range.clip((degrees - Artemis.navx_device.getYaw())/degrees, .05, maxPower);
            Artemis.br.setPower(power);
            Artemis.fr.setPower(power);
            Artemis.bl.setPower(-power);
            Artemis.fl.setPower(-power);
        }
        Artemis.setDrivePower(0);
    }
    public void TurnRightAbsolute(double degrees, double power, LinearOpMode opMode){
        if(!opMode.opModeIsActive())
            Finish(this);
        while(Artemis.navx_device.getYaw() <= degrees && opMode.opModeIsActive()){
            Artemis.br.setPower(-power);
            Artemis.fr.setPower(-power);
            Artemis.bl.setPower(power);
            Artemis.fl.setPower(power);
        }
        Artemis.setDrivePower(0);
    }
    public void TurnRight(double degrees, double power){
        TurnRightAbsolute(degrees, power, this);
    }
    public void TurnRightRelative(double degrees, double power, LinearOpMode opMode){

        if(!opMode.opModeIsActive())
            Finish(this);
        double yaw = Artemis.navx_device.getYaw();
        while(Math.abs(yaw - Artemis.navx_device.getYaw()) < degrees && opMode.opModeIsActive()){
            Artemis.br.setPower(-power);
            Artemis.fr.setPower(-power);
            Artemis.bl.setPower(power);
            Artemis.fl.setPower(power);
        }
        Artemis.setDrivePower(0);
    }
    public void Finish(LinearOpMode opMode){
        Artemis.navx_device.close();
        Artemis.colorSensor.close();
        Artemis.ods.close();
        opMode.stop();
    }
}
