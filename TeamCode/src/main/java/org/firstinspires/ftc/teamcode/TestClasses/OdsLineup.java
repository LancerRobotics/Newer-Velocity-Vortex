package org.firstinspires.ftc.teamcode.TestClasses;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by shlok.khandelwal on 3/9/2017.
 */
@Autonomous(name = "odslineup", group = "tests")
public class OdsLineup extends LinearOpMode {
    Hardware3415 balin = new Hardware3415();

    public void runOpMode() {
        //This code is for blue side auton place robot where the original blue auton code was placed
        //Don't worry about the shooting part comment it out perhaps and test if it is hitting the first line consistently
        //Then if we get the 2nd ods by tomorrow uncomment adjustment strafe it may or may not work
        //GL sorry I won't be there on 3/5
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
        while ((!(balin.ods.getRawLightDetected() >= .5) || !(balin.odsB.getRawLightDetected() >=3)) && opModeIsActive() && !isStopRequested()) {
            if(!(balin.ods.getRawLightDetected()>=.5) && !(balin.odsB.getRawLightDetected()>=3)){
                balin.fr.setPower(-.25);
                balin.br.setPower(-.25);
                balin.fl.setPower(.25);
                balin.bl.setPower(.25);
            }
            else if(balin.ods.getRawLightDetected()>=.5 && !(balin.odsB.getRawLightDetected()>=3)){
                balin.br.setPower(.3);
                balin.bl.setPower(-.3);
                balin.fr.setPower(0);
                balin.fl.setPower(0);
            }
            else if(!(balin.ods.getRawLightDetected()>=.5) && balin.odsB.getRawLightDetected()>=3){
                balin.fr.setPower(-.3);
                balin.fl.setPower(.3);
                balin.br.setPower(0);
                balin.bl.setPower(0);
            }
            else{
                balin.setDrivePower(0);
            }
            telemetry.addData("ods", balin.ods.getRawLightDetected());
            telemetry.addData("odsB", balin.odsB.getRawLightDetected());
            telemetry.addData("fl", balin.fl.getPower());
            telemetry.addData("fr", balin.fr.getPower());
            telemetry.addData("br", balin.br.getPower());
            telemetry.addData("bl", balin.bl.getPower());
            telemetry.update();

        }
    }
}
