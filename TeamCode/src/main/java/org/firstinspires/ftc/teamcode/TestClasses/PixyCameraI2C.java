package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by andrew.keenan on 4/3/2017.
 */

public class PixyCameraI2C extends LinearOpMode {
    public I2cDevice PIXY = null;
    public I2cDeviceSynch PIXYReader = null;
    byte[] PIXYCacheX;
    public static final int PIXY_REG_START = 0x04;
    public static final int PIXY_READ_LENGTH = 2;

    public void runOpMode() {
        PIXY = hardwareMap.i2cDevice.get("pixy");
        PIXYReader = new I2cDeviceSynchImpl(PIXY, I2cAddr.create8bit(0x98), false);
        PIXYReader.engage();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            PIXYCacheX = PIXYReader.read(PIXY_REG_START, PIXY_READ_LENGTH);
            double pixyDataX = PIXYCacheX[0] & 0xFF;
            telemetry.addData("Pixy Data", pixyDataX);
        }
    }
}
