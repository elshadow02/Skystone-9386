package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Teleop.EEHardware;

@Autonomous(name="DownTest", group ="Concept")
public class DownTest extends LinearOpMode {

    EEHardware bot = new EEHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();

        while (!bot.down.isPressed()){
            bot.lift.setPower(-1);
            sleep(825);
            bot.lift.setPower(0);
            break;
        }
    }
}
