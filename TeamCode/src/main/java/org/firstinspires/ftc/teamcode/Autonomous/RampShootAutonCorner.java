package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by dina.brustein on 3/11/2017.
 */
@Autonomous(name = "Corner Start Ramp Park", group= "tests")
public class RampShootAutonCorner extends LinearOpMode {
    Hardware3415 balin = new Hardware3415();
    public void runOpMode(){
        balin.moveStraightnew(8, this);
        balin.shoot(1.0);
        sleep(600);
        balin.shoot(0);
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
        sleep(200);
        balin.gyroAngle(10,.25,this);
        balin.fr.setPower(0.9);
        balin.fl.setPower(-0.9);
        balin.br.setPower(0.9);
        balin.bl.setPower(-0.9);
        sleep(1500);
        balin.setDrivePower(0);
    }
}
