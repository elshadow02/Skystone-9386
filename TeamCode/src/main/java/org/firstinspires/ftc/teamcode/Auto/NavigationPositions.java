/*
    FTC Team 9386 Elmer and Elsie Robotics Skystone
 */
package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Auto.CoordinatePosition;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleop.EEHardware;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;



import org.firstinspires.ftc.teamcode.Auto.SkystoneFPS;

/**
 * Main Coordinate Position Class.
 *
 * Made by Ethan. Skystone 2019-20 Season
 */
@TeleOp(name="nav pos", group="TeleOp")
public class NavigationPositions extends LinearOpMode
{
    public CoordinatePosition skystone1 = new CoordinatePosition();
    public CoordinatePosition skystone2 = new CoordinatePosition();
    public CoordinatePosition skystone3 = new CoordinatePosition();
    public CoordinatePosition skystone4 = new CoordinatePosition();
    public CoordinatePosition skystone5 = new CoordinatePosition();
    public CoordinatePosition skystone6 = new CoordinatePosition();

    public CoordinatePosition redFoundation = new CoordinatePosition();
    public CoordinatePosition blueFoundation = new CoordinatePosition();

    public SkystoneFPS fps = new SkystoneFPS();

    final float mmPerInch        = 25.4f;
    final float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    final float mmStonePosition  = (-mmFTCFieldWidth/2) + (4*mmPerInch);
    final float mmStoneLength    = 8 * mmPerInch;
    final float mmFoundationXPosition = 23.75f * mmPerInch;
    final float mmFoundationYPosition = (mmFTCFieldWidth/2)-(4*mmPerInch);

    float xPosition = 0;
    float yPosition = 0;

    VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */

    //SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

    WebcamName webcamName;

    VuforiaLocalizer.Parameters parameters;


    /* Constructor */
    public NavigationPositions(){

    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AQapn2P/////AAABmRxgWZJT7kKXqyyOQcN4AidYCoBPE/BzcDQASgQ+5iM8wvdkBbzR8qPTDddZSHUp0VsvcAKm8KwIWUElQXmamu9q/iTAzYdJ+aFu/b+2Zyf/+9ZbluiqiSPLptgv/ocKQqzY6nCFoV4qzSFGhH45oRThSBuKmWxrAGJHIo1mnrRdSuyuIOf8JIqo9J9bdqApsVZOSEiuglT7YNQE3DEBAsS9xCLLu8lfn/SvpgzaEy+pBOoehvJOCQ6QabYUz2ZiaaB0CrOLkPjP7OnafVAoo+NZ6vOOqfwRfqEwWUT/YYOoTn8zJLD0+tBdqSZkdVn5sT46CxfZFz1NHfd5RvHzRBcPrI3iB6lXtvCuS8csqLL0";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        fps.initialize(parameters);

        int stopButton = 0;

        waitForStart();

        while (opModeIsActive() && stopButton != 1) {
            resetXYPositions();

            telemetryUpdate();

            if (gamepad1.a){
                stopButton = 1;
                break;
            }

            sleep(7000);
        }
    }

    public void initialize(){
        skystone1.setXY(24*mmPerInch, mmStonePosition);
        skystone2.setXY(24*mmPerInch,mmStonePosition + mmStoneLength);
        skystone3.setXY(24*mmPerInch,mmStonePosition + (2*mmStoneLength));
        skystone4.setXY(24*mmPerInch,mmStonePosition + (3*mmStoneLength));
        skystone5.setXY(24*mmPerInch,mmStonePosition + (4*mmStoneLength));
        skystone6.setXY(24*mmPerInch,mmStonePosition + (5*mmStoneLength));

        blueFoundation.setXY(-mmFoundationXPosition, mmFoundationYPosition);
        redFoundation.setXY(mmFoundationXPosition, mmFoundationYPosition);
    }
    public void VuforiaInit(){
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AQapn2P/////AAABmRxgWZJT7kKXqyyOQcN4AidYCoBPE/BzcDQASgQ+5iM8wvdkBbzR8qPTDddZSHUp0VsvcAKm8KwIWUElQXmamu9q/iTAzYdJ+aFu/b+2Zyf/+9ZbluiqiSPLptgv/ocKQqzY6nCFoV4qzSFGhH45oRThSBuKmWxrAGJHIo1mnrRdSuyuIOf8JIqo9J9bdqApsVZOSEiuglT7YNQE3DEBAsS9xCLLu8lfn/SvpgzaEy+pBOoehvJOCQ6QabYUz2ZiaaB0CrOLkPjP7OnafVAoo+NZ6vOOqfwRfqEwWUT/YYOoTn8zJLD0+tBdqSZkdVn5sT46CxfZFz1NHfd5RvHzRBcPrI3iB6lXtvCuS8csqLL0";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        fps.initialize(parameters);
    }

//    public void travelToPosition(double targetX, double targetY, double targetHeading){
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(targetX - getCurrentXPosition(), targetY - getCurrentYPosition(), targetHeading))
//                        .build()
//        );
//    }

//    public void travelToObject (CoordinatePosition object, double targetHeading){
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(object.XCoordinate - getCurrentXPosition(), object.YCoordinate - getCurrentYPosition(), targetHeading))
//                        .build()
//        );
//    }

    public void resetXYPositions(){
        fps.vuMarkInit(5.0, this.xPosition, this.yPosition, parameters);
    }

    public double getCurrentXPosition(){
        return this.xPosition;
    }

    public double getCurrentYPosition(){
        return this.yPosition;
    }

    public void telemetryUpdate(){
        telemetry.addData("current X Pos: ", this::getCurrentXPosition);
        telemetry.addData("current Y Pos: ", this::getCurrentYPosition);
    }
}