package org.firstinspires.ftc.teamcode.Teleop;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by Paspuleti on 3/31/2017.
 */
@TeleOp(name = "Test Teleop", group = "tests")
public class Sentry extends LinearOpMode {
    Hardware3415 Balin = new Hardware3415();

    public void runOpMode() {
        Balin.init(hardwareMap, false);

        // Send telemetry message to signify Balin waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if(Math.abs(gamepad2.right_stick_y) > .15) {
            Balin.lift(gamepad2.right_stick_y);
        }
    }
}
