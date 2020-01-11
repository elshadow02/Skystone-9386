package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Limit Switch Test")
public class LimitSwitchTest extends LinearOpMode {

    @Override
    public void runOpMode () {

        DigitalChannel dig;

        dig = hardwareMap.get(DigitalChannel.class, "dig");

        telemetry.addData("We are ready, ", "aren't we?");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Is it pressed? ", dig.getState());
            telemetry.update();
        }
    }

}
