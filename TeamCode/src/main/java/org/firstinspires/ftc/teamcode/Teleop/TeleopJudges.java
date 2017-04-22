package org.firstinspires.ftc.teamcode.Teleop;

/**
 * Created by Paspuleti on 4/19/2017.
 */

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

//Needs testing

@TeleOp (name="Judges Teleop", group= "Worlds")
public class TeleopJudges extends LinearOpMode {

    Hardware3415 Artemis = new Hardware3415();

    public void runOpMode() {
        Artemis.init(hardwareMap, false);

        while (opModeIsActive()) {
            if (gamepad1.x) { //Spin ring, and moving the flap
                while (!gamepad1.y) { //we can switch this for a for loop
                    Artemis.rotate.setPower(.86);
                    Artemis.flap.setPosition(1);
                    Artemis.flap.setPosition(0);
                }
                Artemis.rotate.setPower(0);
            }

            if (gamepad1.a) {//Spins collector
                while (!gamepad1.b) {
                    Artemis.collector.setPower(.99);
                }
                Artemis.collector.setPower(0);
            }
        }
    }
}
