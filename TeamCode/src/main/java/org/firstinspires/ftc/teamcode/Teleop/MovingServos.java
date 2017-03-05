package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by david.lin on 2/20/2017.
 */

@TeleOp(name = "MovingServos", group = "Competition")

public class MovingServos extends LinearOpMode {
    Hardware3415 balin = new Hardware3415();
    public void runOpMode() {
        balin.init(hardwareMap, false);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                balin.clampLeft.setPosition(0);
                balin.clampRight.setPosition(1);
            }
            else if(gamepad1.y) {
                balin.clampLeft.setPosition(122.0/255);
                balin.clampRight.setPosition(123.0/255);
            }
        }
    }
}
