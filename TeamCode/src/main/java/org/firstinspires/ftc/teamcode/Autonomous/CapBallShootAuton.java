package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by andrew.keenan on 2/26/2017.
 */
@Autonomous(name = "hit and shoot", group= "tests")
public class CapBallShootAuton extends LinearOpMode {
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        balin.init(hardwareMap, true);

        balin.shoot(1);
        sleep(500);
        sleep(10000);
        balin.setDrivePower(.3);
        sleep(3000);
        balin.setDrivePower(0);

    }
}
