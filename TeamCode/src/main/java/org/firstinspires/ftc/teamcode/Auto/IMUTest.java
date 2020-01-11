package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Teleop.EEHardware;

@TeleOp(name="Imu Test", group="test")
public class IMUTest extends LinearOpMode {

    EEHardware bot = new EEHardware();

    @Override
    public void runOpMode(){
        bot.init(hardwareMap);
        Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        int count = 0;
        waitForStart();
        while (opModeIsActive()){
            orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            count++;

            telemetry.addData("Count", count);
            telemetry.addData("First", orientation.firstAngle);
            telemetry.addData("Second", orientation.secondAngle);
            telemetry.addData("Third", orientation.thirdAngle);
            telemetry.update();
        }
    }
}
