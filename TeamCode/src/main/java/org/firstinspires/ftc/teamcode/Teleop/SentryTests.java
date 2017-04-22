package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriversAndHardware.Hardware3415;

/**
 * Created by Paspuleti on 4/15/2017.
 */
@TeleOp(name = "Lift Test", group = "Tests")
public class SentryTests extends LinearOpMode{
    Hardware3415 Artemis = new Hardware3415();

    public void runOpMode(){
        Artemis.init(hardwareMap, false);
        // Send telemetry message to signify Balin waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
        //Cap Ball Lift Code:

            if(gamepad2.left_trigger > .15){
                Artemis.lift(-.86);
            }
            else if(gamepad2.right_trigger > .15){
                Artemis.lift(.86);
            } else{
                Artemis.lift(0);
            }
            if(gamepad2.a){
                Artemis.clampLeft.setPosition(Artemis.LEFT_CLAMP_CLAMP);
                Artemis.clampRight.setPosition(Artemis.RIGHT_CLAMP_CLAMP);
            }
            if(gamepad2.y){
                Artemis.clampLeft.setPosition(Artemis.LEFT_CLAMP_UP);
                Artemis.clampRight.setPosition(Artemis.RIGHT_CLAMP_UP);
            }

            if(Math.abs(gamepad2.left_stick_y)> .15) {
                Artemis.shoot(gamepad2.left_stick_y*.85);
                telemetry.addData("power shooter", gamepad1.left_stick_y);
                telemetry.update();
            }

            else
                Artemis.shoot(0);
            if(gamepad1.right_trigger>.15){
                Artemis.collector.setPower(1.0);
            }
            else if(gamepad1.left_trigger>.15){
                Artemis.collector.setPower(-1.0);
            }
            else{
                Artemis.collector.setPower(0);
            }

        }

        //Shooter Code
    }
}
