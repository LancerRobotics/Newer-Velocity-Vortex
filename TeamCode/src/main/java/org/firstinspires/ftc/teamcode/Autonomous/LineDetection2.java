package org.firstinspires.ftc.teamcode.Autonomous;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by shlok.khandelwal on 3/2/2017.
 */
@Autonomous(name = "ODS TEST", group = "tests")
public class LineDetection2 extends LinearOpMode{
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        balin.init(hardwareMap, true);
        balin.navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get(balin.cdim),
                balin.NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                balin.NAVX_DEVICE_UPDATE_RATE_HZ);
        waitForStart();
        while(true){

            double lightF = balin.ods.getRawLightDetected();

            telemetry.addData("Light Detected ods:", lightF);
            telemetry.addData("Red", balin.colorSensor.red());
            telemetry.addData("Blue", balin.colorSensor.blue());
            telemetry.addData("Green", balin.colorSensor.green());
            telemetry.addData("Sonar 2", balin.readSonar2());
            telemetry.addData("Gyro Calibrating", balin.navx_device.isCalibrating());
            telemetry.update();
        }


    }


}
