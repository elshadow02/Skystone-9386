package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

//import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
//import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really foundationLeft the tuning.
 */
@TeleOp(name="nav foundationLeft", group = "TeleOp")
public class NavigationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        telemetry.addData("we are ", "1");
        telemetry.update();

        sleep(2000);

        NavigationPositions nav = new NavigationPositions();
        sleep(2000);

        telemetry.addData("we are ", "2");
        telemetry.update();
        sleep(2000);

        int stopButton = 0;

        nav.VuforiaInit();
        sleep(2000);

        telemetry.addData("we are ", "3");
        telemetry.update();
        sleep(2000);

        waitForStart();

        while (opModeIsActive() && stopButton != 1) {

            telemetry.addData("we are ", "4");
            telemetry.update();
            sleep(2000);

            nav.resetXYPositions();
            sleep(2000);

            telemetry.addData("we are ", "5");
            telemetry.update();

            nav.telemetryUpdate();

            if (gamepad1.a){
                stopButton = 1;
                break;
            }

            sleep(7000);
        }
    }
}
