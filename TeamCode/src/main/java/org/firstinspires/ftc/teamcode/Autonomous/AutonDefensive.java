package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by dina.brustein on 3/11/2017.
 */

public class AutonDefensive extends LinearOpMode {
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        sleep(10000);
        balin.fl.setPower(0.9);
        balin.br.setPower(0.9);
        sleep(500);
        balin.setDrivePower(0);
    }
}
