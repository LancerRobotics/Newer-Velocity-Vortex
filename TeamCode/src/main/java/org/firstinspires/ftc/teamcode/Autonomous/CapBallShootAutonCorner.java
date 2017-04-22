package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by dina.brustein on 3/11/2017.
 */

@Autonomous(name = "Corner Start Center Park", group = "Auton")

public class CapBallShootAutonCorner extends LinearOpMode {
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        balin.moveStraightnew(16, this);
        balin.shoot(1.0);
        sleep(600);
        balin.shoot(0);
        //balin.door.setPosition(balin.DOOR_OPEN);
        balin.collector.setPower(1.0);
        sleep(2000);
        balin.collector.setPower(0);
        //balin.door.setPosition(balin.DOOR_CLOSED);
        sleep(500);
        balin.shoot(1.0);
        sleep(600);
        balin.shoot(0);
        sleep(200);
        balin.setDrivePower(0.7);
        sleep(1500);
        balin.setDrivePower(0);
    }
}

