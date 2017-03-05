package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by shlok.khandelwal on 3/2/2017.
 */

public class LineDetection2 extends LinearOpMode{
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        balin.init(hardwareMap, true);
        waitForStart();
        while(!(balin.ods.getRawLightDetected()>=.5)&& opModeIsActive()){ //this ods should be the front most ods
            balin.setDrivePower(.3);
        }
        balin.setDrivePower(0);
        sleep(500);
        while(!(balin.odsB.getRawLightDetected()>=.5)&& opModeIsActive()){
            balin.turn(0.3);
        }
        balin.setDrivePower(0);

    }


}
