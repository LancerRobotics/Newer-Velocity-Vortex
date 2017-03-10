package org.firstinspires.ftc.teamcode.Autonomous;

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
        waitForStart();
        while(opModeIsActive()){
            double light = balin.odsB.getRawLightDetected();
            double lightF = balin.ods.getRawLightDetected();
            telemetry.addData("Light detected odsB:", light);
            telemetry.addData("Light Detected ods:", lightF);
            telemetry.update();
        }


    }


}
