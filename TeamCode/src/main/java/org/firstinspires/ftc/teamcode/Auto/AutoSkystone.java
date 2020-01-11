/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Teleop.EEHardware;
import org.openftc.revextensions2.ExpansionHubEx;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
Created by: Ethan Lanting
Date: 10/12/19

Skystone Field Positioning System (FPS) using a Webcam and VuMarks.
 */

@Autonomous(name="Auto Skystone", group ="Concept")
public class AutoSkystone extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    EEHardware robot = new EEHardware();

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);

    double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    double TICKS_PER_ROTATION  = MOTOR_CONFIG.getTicksPerRev();
    double TICKS_PER_INCH = 40;
//    double TICKS_PER_INCH = 537.6/WHEEL_CIRCUMFERENCE;
//    double TICKS_PER_INCH = 537.6 / 14.75;

    public double DRIVE_kP = 1.0/20;
    public double DRIVE_kI = 1.0/40;
    public double DRIVE_kD = 1.0/15;

    private double TURN_kP = 0.05;  // increase this number to increase responsiveness. decrease this number to decrease oscillation
    private double TURN_kI = 0.01;  // increase this number to decrease steady state error (controller stops despite error not equalling 0)
    private double TURN_kD = 0.025; // increase this number to increase the "slowdown" as error grows smaller

    private DcMotor wheel1;
    private DcMotor wheel2;
    private DcMotor wheel3;
    private DcMotor wheel4;

    //SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

    OpenGLMatrix lastLocation = null;
//
//    /**
//     * @see #captureFrameToFile()
//     */
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;


    //VuforiaLocalizer vuforia;

    final float mmPerInch = 25.4f;
    final float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;

    final float mmStonePosition  = (-142/2) + 4f;
    final float mmStoneLength    = 8;

    final float mmFoundationXPosition = 23.75f;
    final float mmFoundationYPosition = 142f/4f;
    final float mmFoundation2XPosition = (23.75f) + (234.95f/25.4f);
    final float mmFoundation2YPosition = (142f/2f) - 977.9f/25.4f;

    float targetX = 0;
    float targetY = 0;

    final double WHEEL_RADIUS = 2 * 25.4;
    final double ENCODER_TICKS = 537.6;

    double xDiff = 0;

    CoordinatePosition blueSkystone1 = new CoordinatePosition();
    CoordinatePosition blueSkystone2 = new CoordinatePosition();
    CoordinatePosition blueSkystone3 = new CoordinatePosition();
    CoordinatePosition blueSkystone4 = new CoordinatePosition();
    CoordinatePosition blueSkystone5 = new CoordinatePosition();
    CoordinatePosition blueSkystone6 = new CoordinatePosition();

    CoordinatePosition redSkystone1 = new CoordinatePosition();
    CoordinatePosition redSkystone2 = new CoordinatePosition();
    CoordinatePosition redSkystone3 = new CoordinatePosition();
    CoordinatePosition redSkystone4 = new CoordinatePosition();
    CoordinatePosition redSkystone5 = new CoordinatePosition();
    CoordinatePosition redSkystone6 = new CoordinatePosition();

    CoordinatePosition targetSkystone1 = new CoordinatePosition();
    CoordinatePosition targetSkystone2 = new CoordinatePosition();

    CoordinatePosition blueFoundation = new CoordinatePosition();
    CoordinatePosition redFoundation = new CoordinatePosition();
    CoordinatePosition blueFoundation2 = new CoordinatePosition();
    CoordinatePosition redFoundation2 = new CoordinatePosition();

    private BNO055IMU imu;
    private ExpansionHubEx hub;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    //WebcamName webcamName;
    float currentX = 0, currentY = 0;
    float yPosition = 0, xPosition = 0;
    float xDifference = 0, yDifference = 0;

    @Override
    public void runOpMode() {

        /*
         * Retrieve the camera we are to use.
         */

        robot.init(hardwareMap);

        wheel1 = robot.frontRight;
        wheel2 = robot.frontLeft;
        wheel3 = robot.backLeft;
        wheel4 = robot.backRight;


        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //parameters.vuforiaLicenseKey = "AQapn2P/////AAABmRxgWZJT7kKXqyyOQcN4AidYCoBPE/BzcDQASgQ+5iM8wvdkBbzR8qPTDddZSHUp0VsvcAKm8KwIWUElQXmamu9q/iTAzYdJ+aFu/b+2Zyf/+9ZbluiqiSPLptgv/ocKQqzY6nCFoV4qzSFGhH45oRThSBuKmWxrAGJHIo1mnrRdSuyuIOf8JIqo9J9bdqApsVZOSEiuglT7YNQE3DEBAsS9xCLLu8lfn/SvpgzaEy+pBOoehvJOCQ6QabYUz2ZiaaB0CrOLkPjP7OnafVAoo+NZ6vOOqfwRfqEwWUT/YYOoTn8zJLD0+tBdqSZkdVn5sT46CxfZFz1NHfd5RvHzRBcPrI3iB6lXtvCuS8csqLL0";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        //parameters.cameraName = webcamName;

        /**
         * Instantiate the Vuforia engine
         */
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
         * see.
         */
        //vuforia.enableConvertFrameToBitmap();

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        //setTicksPerInch(WHEEL_RADIUS, ENCODER_TICKS);

        setCoords();

        hub = hardwareMap.get(ExpansionHubEx .class, "hub");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters iMUParameters = new BNO055IMU.Parameters();
        iMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(iMUParameters);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

//        if (is)
//
//        vuMarkInit(5.0, currentX, currentY, parameters);
//
//        int fieldPos = getFieldStartPosition();

        Orientation startingOrientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //drive.setPoseEstimate(new Pose2d(currentX/25.4, currentY/25.4, startingOrientation.thirdAngle));

        robot.setCords(currentX/25.4, currentY/25.4);

        telemetry.addData("Current X: ", currentX);
        telemetry.addData("Current Y: ", currentY);
       // telemetry.addData("Field Positon", fieldPos);
        telemetry.update();

        sleep (1000);

//        if (fieldPos == 1){
//            redBuildingZone(parameters);
//        }
//        else if (fieldPos == 2){
//            blueBuildingZone(parameters);
//        }
//        else if (fieldPos == 3){
//            blueLoadingZone(parameters);
//        }
//        else if (fieldPos == 4){
//            redLoadingZone(parameters);
//        }
//        else{
//            telemetry.addLine("Something went terribly, terribly wrong...");
//            telemetry.update();
//        }

//        findSkystone(parameters);
//
//        CoordinatePosition target = determineSkystone();
//
//        telemetry.addData("Target: ", target);
//        telemetry.update();
//
//        sleep (5000);

        //gyroDrive(0, -1.0, 26, 5.0);


        gyroDrive(0, 1.0, 9.0, 5.0);
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    public String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the camera is actually seeing and so assist in camera
     * aiming and alignment.
     */
//    public void captureFrameToFile() {
////        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
////            @Override
////            public void accept(Frame frame) {
////                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
////                if (bitmap != null) {
////                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
////                    try {
////                        FileOutputStream outputStream = new FileOutputStream(file);
////                        try {
////                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
////                        } finally {
////                            outputStream.close();
////                            telemetry.log().add("captured %s", file.getName());
////                        }
////                    } catch (IOException e) {
////                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
////                    }
////                }
////            }
////        }));
////    }

    public void setCoords(){
        blueSkystone1.setXY(24, mmStonePosition);
        blueSkystone2.setXY(24, mmStonePosition + mmStoneLength);
        blueSkystone3.setXY(24, mmStonePosition + (2*mmStoneLength));
        blueSkystone4.setXY(24, mmStonePosition + (3*mmStoneLength));
        blueSkystone5.setXY(24, mmStonePosition + (4*mmStoneLength));
        blueSkystone6.setXY(24, mmStonePosition + (5*mmStoneLength));

        redSkystone1.setXY(-24, mmStonePosition);
        redSkystone2.setXY(-24, mmStonePosition + mmStoneLength);
        redSkystone3.setXY(-24, mmStonePosition + (2*mmStoneLength));
        redSkystone4.setXY(-24, mmStonePosition + (3*mmStoneLength));
        redSkystone5.setXY(-24, mmStonePosition + (4*mmStoneLength));
        redSkystone6.setXY(-24, mmStonePosition + (5*mmStoneLength));

        blueFoundation.setXY(-mmFoundationXPosition, mmFoundationYPosition);
        redFoundation.setXY(mmFoundationXPosition, mmFoundationYPosition);
        blueFoundation2.setXY(-mmFoundation2XPosition, mmFoundation2YPosition);
        redFoundation2.setXY(mmFoundation2XPosition, mmFoundation2YPosition);
    }

//    public void travelToObject(CoordinatePosition pos, double angle){
//        Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//        drive.followTrajectorySync(drive.trajectoryBuilder().
//                splineTo(new Pose2d(pos.getXCoordinate(), pos.getYCoordinate(), angle))
//                .build());
//
//        while (opModeIsActive() && drive.isBusy()) {
//            drive.update();
//        }
//    }

//    public void travelToPosition(double targetX, double targetY, double angle){
//        drive.followTrajectorySync(drive.trajectoryBuilder().
//                splineTo(new Pose2d(targetX, targetY, angle))
//                .build());
//
//        while (opModeIsActive() && drive.isBusy()) {
//            drive.update();
//        }
//    }

//    public void travelStraight(double distance){
//        Trajectory trajectory = drive.trajectoryBuilder()
//                .forward(distance)
//                .build();
//
//        drive.followTrajectorySync(trajectory);
//
//        while (opModeIsActive() && drive.isBusy()) {
//            drive.update();
//        }
//    }

//    public void travelBackwards(double distance){
//        Trajectory trajectory = drive.trajectoryBuilder()
//                .back(distance)
//                .build();
//
//        drive.followTrajectorySync(trajectory);
//
//        while (opModeIsActive() && drive.isBusy()) {
//            drive.update();
//        }
//    }

//    public void strafe(double distance, boolean leftDirection){
//        Trajectory trajectory;
//
//        if (leftDirection == true) {
//            trajectory = drive.trajectoryBuilder()
//                    .strafeLeft(distance)
//                    .build();
//        }
//        else{
//            trajectory = drive.trajectoryBuilder()
//                    .strafeRight(distance)
//                    .build();
//        }
//
//        drive.followTrajectorySync(trajectory);
//
//        while (opModeIsActive() && drive.isBusy()) {
//            drive.update();
//        }
//    }
//
//    public void turn(double angle){
//        drive.turnSync(Math.toRadians(angle));
//    }

    //Constructor
    public AutoSkystone() {

    }

//    public void initialize(VuforiaLocalizer.Parameters parameters){
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        /**
//         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
//         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
//         * see.
//         */
//        vuforia.enableConvertFrameToBitmap();
//
//        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
//    }

//    public void vuMarkInit(double maxTime, float xPosition, float yPosition, VuforiaLocalizer.Parameters parameters) {
//
//        /*
//        Define VuMarks.
//        Rear/Back is in the Building Zone.
//        Front is in the Loading Zone.
//         */
//        VuforiaTrackables skystone = vuforia.loadTrackablesFromAsset("Skystone");
//        VuforiaTrackable bridgeBlueBack = skystone.get(1);
//        bridgeBlueBack.setName("bridgeBlueBack");
//
//        VuforiaTrackable bridgeRedBack = skystone.get(2);
//        bridgeRedBack.setName("bridgeRedBack");
//
//        VuforiaTrackable bridgeRedFront = skystone.get(3);
//        bridgeRedBack.setName("bridgeRedFront");
//
//        VuforiaTrackable bridgeBlueFront = skystone.get(4);
//        bridgeRedBack.setName("bridgeBlueFront");
//
//        VuforiaTrackable RedPerimeterBack = skystone.get(5);
//        bridgeRedBack.setName("RedPerimeterBack");
//
//        VuforiaTrackable RedPerimeterFront = skystone.get(6);
//        bridgeRedBack.setName("RedPerimeterFront");
//
//        VuforiaTrackable FrontPerimeterRed = skystone.get(7);
//        bridgeRedBack.setName("FrontPerimeterRed");
//
//        VuforiaTrackable FrontPerimeterBlue = skystone.get(8);
//        bridgeRedBack.setName("FrontPerimeterBlue");
//
//        VuforiaTrackable BluePerimeterFront = skystone.get(9);
//        bridgeRedBack.setName("BluePerimeterFront");
//
//        VuforiaTrackable BluePerimeterBack = skystone.get(10);
//        bridgeRedBack.setName("BluePerimeterBack");
//
//        VuforiaTrackable RearPerimeterBlue = skystone.get(11);
//        bridgeRedBack.setName("RearPerimeterBlue");
//
//        VuforiaTrackable RearPerimeterRed = skystone.get(12);
//        bridgeRedBack.setName("RearPerimeterRed");
//
//        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(skystone);
//
//        //Convert field measurements to mm because Skystone XML file data uses mm.
//        // the FTC field is ~11'10" (142") center-to-center of the glass panels
//        final float vumarkHeight = 5.75f * mmPerInch;
//        final float vumarkDistanceFromWallCenter = 35 * mmPerInch;
//        final float bridgeVumarkXDistanceFromOrigin = 24 * mmPerInch;
//        final float bridgeVumarkYDistanceFromOrigin = 9 * mmPerInch;
//
//        final float xOrigin = 0.0f;
//        final float yOrigin = 0.0f;
//        final float zOrigin = 0.0f;
//
//        final float X_MAX = mmFTCFieldWidth / 2;
//        final float Y_MAX = mmFTCFieldWidth / 2;
//        final float Z_MAX = mmFTCFieldWidth / 2;
//
//        final float X_MIN = -mmFTCFieldWidth / 2;
//        final float Y_MIN = -mmFTCFieldWidth / 2;
//        final float Z_MIN = 0.0f;
//
//
//        //Define Rear Perimeter Target 1 position on field.
//        OpenGLMatrix RearPerimeterBlueLocation = OpenGLMatrix
//                .translation(-vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 180, 0));
//        RearPerimeterBlue.setLocationFtcFieldFromTarget(RearPerimeterBlueLocation);
//        RobotLog.ii(TAG, "Rear Perimeter Blue=%s", format(RearPerimeterBlueLocation));
//
//        //Define Rear Perimeter Target 2 position on field.
//        OpenGLMatrix RearPerimeterRedLocation = OpenGLMatrix
//                .translation(vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 180, 0));
//        RearPerimeterRed.setLocationFtcFieldFromTarget(RearPerimeterRedLocation);
//        RobotLog.ii(TAG, "Rear Perimeter Red=%s", format(RearPerimeterRedLocation));
//
//        //Define Front Perimeter Target 2 position on field.
//        OpenGLMatrix FrontPerimeterBlueLocation = OpenGLMatrix
//                .translation(-vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        FrontPerimeterBlue.setLocationFtcFieldFromTarget(FrontPerimeterBlueLocation);
//        RobotLog.ii(TAG, "Front Perimeter Blue=%s", format(FrontPerimeterBlueLocation));
//
//        //Define Front Perimeter Target 1 position on field.
//        OpenGLMatrix FrontPerimeterRedLocation = OpenGLMatrix
//                .translation(vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        FrontPerimeterRed.setLocationFtcFieldFromTarget(FrontPerimeterRedLocation);
//        RobotLog.ii(TAG, "Front Perimeter Red=%s", format(FrontPerimeterRedLocation));
//
//        //Define Red Perimeter Target 1 position on field.
//        OpenGLMatrix RedPerimeterBackLocation = OpenGLMatrix
//                .translation(mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, -90, 0));
//        RedPerimeterBack.setLocationFtcFieldFromTarget(RedPerimeterBackLocation);
//        RobotLog.ii(TAG, "Red Perimeter Back=%s", format(RedPerimeterBackLocation));
//
//        //Define Red Perimeter Target 2 position on field.
//        OpenGLMatrix RedPerimeterFrontLocation = OpenGLMatrix
//                .translation(mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, -90, 0));
//        RedPerimeterFront.setLocationFtcFieldFromTarget(RedPerimeterFrontLocation);
//        RobotLog.ii(TAG, "Red Perimeter Front=%s", format(RedPerimeterFrontLocation));
//
//        //Define Blue Perimeter Target 2 position on field.
//        OpenGLMatrix BluePerimeterBackLocation = OpenGLMatrix
//                .translation(-mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        BluePerimeterBack.setLocationFtcFieldFromTarget(BluePerimeterBackLocation);
//        RobotLog.ii(TAG, "Blue Perimeter Back=%s", format(BluePerimeterBackLocation));
//
//        //Define Blue Perimeter Target 1 position on field.
//        OpenGLMatrix BluePerimeterFrontLocation = OpenGLMatrix
//                .translation(-mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        BluePerimeterFront.setLocationFtcFieldFromTarget(BluePerimeterFrontLocation);
//        RobotLog.ii(TAG, "Blue Perimeter Front=%s", format(BluePerimeterFrontLocation));
//
//        //Define Bridge Blue Back position on field.
//        OpenGLMatrix BridgeBlueBackLocation = OpenGLMatrix
//                .translation(-bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        bridgeBlueBack.setLocationFtcFieldFromTarget(BridgeBlueBackLocation);
//        RobotLog.ii(TAG, "Bridge Blue Back=%s", format(BridgeBlueBackLocation));
//
//        //Define Red Bridge Back position on field.
//        OpenGLMatrix BridgeRedBackLocation = OpenGLMatrix
//                .translation(bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        bridgeRedBack.setLocationFtcFieldFromTarget(BridgeRedBackLocation);
//        RobotLog.ii(TAG, "Red Bridge Back=%s", format(BridgeRedBackLocation));
//
//        //Define Bridge Blue Front position on field.
//        OpenGLMatrix BridgeBlueFrontLocation = OpenGLMatrix
//                .translation(-bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 180, 0));
//        bridgeBlueFront.setLocationFtcFieldFromTarget(BridgeBlueFrontLocation);
//        RobotLog.ii(TAG, "Blue Bridge Front=%s", format(BridgeBlueFrontLocation));
//
//        //Define Red Bridge Front position on field.
//        OpenGLMatrix BridgeRedFrontLocation = OpenGLMatrix
//                .translation(bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 180, 0));
//        bridgeRedFront.setLocationFtcFieldFromTarget(BridgeRedFrontLocation);
//        RobotLog.ii(TAG, "Red Bridge Front=%s", format(BridgeRedFrontLocation));
//
//
//        // Define position of camera in relation to the robot.
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(mmBotWidth, 6f*25.4f, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZY,
//                        AngleUnit.DEGREES, 90, 45, 0));
//        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));
//
//        /**
//         * Let the trackable listeners we care about know where the camera is. We know that each
//         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
//         * we have not ourselves installed a listener of a different type.
//         */
//        ((VuforiaTrackableDefaultListener) RearPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) RearPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) FrontPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) FrontPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) BluePerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) BluePerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) RedPerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) RedPerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) bridgeBlueBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) bridgeBlueFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) bridgeRedBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener) bridgeRedFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//
//        /** Start tracking the data sets we care about. */
//        skystone.activate();
//
////        MatrixF robotXTranslation;
////        MatrixF robotYTranslation;
//
//        boolean buttonPressed = false;
//        double startTime = time;
//        while ((time - startTime) < maxTime) {
//
//            if (buttonPressed == true) {
//                captureFrameToFile();
//                buttonPressed = false;
//            }
//
//            for (VuforiaTrackable trackable : allTrackables) {
//                /**
//                 * getUpdatedRobotLocation() will return null if no new information is available since
//                 * the last time that call was made, or if the trackable is not currently visible.
//                 * getRobotLocation() will return null if the trackable is not currently visible.
//                 */
//
//                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
//
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//            }
//
////            robotXTranslation = lastLocation.getTranslation().get(0);
////            robotYTranslation = lastLocation.getTranslation().get(1);
//
//            /**
//             * Provide feedback as to where the robot was last located (if we know).
//             */
//            if (lastLocation != null) {
//                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
//                telemetry.addData("Pos", format(lastLocation));
//                telemetry.addData(" ", "");
//                telemetry.addData("X-Value: ", lastLocation.getTranslation().get(0));
//                telemetry.addData(" ", "");
//                telemetry.addData("Y-Value: ", lastLocation.getTranslation().get(1));
//            } else {
//                telemetry.addData("Pos", "Unknown");
//            }
//            telemetry.update();
//        }
//
//        currentX = lastLocation.getTranslation().get(0);
//        currentY = lastLocation.getTranslation().get(1);
//
//        skystone.deactivate();
//    }

//    public void findSkystone(VuforiaLocalizer.Parameters parameters){
//        VuforiaTrackables skystones = vuforia.loadTrackablesFromAsset("Skystone");
//        VuforiaTrackable skystone = skystones.get(0);
//        skystone.setName("skystone");
//
//        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(skystones);
//
//        boolean blue = false;
//        int side = getFieldStartPosition();
//
//        if (side == 2 || side == 3){
//            blue = true;
//        }
//
//        if (blue == true) {
//            OpenGLMatrix skystoneLocation = OpenGLMatrix
//                    .translation(0, 0, 4 * mmPerInch)
//                    .multiplied(Orientation.getRotationMatrix(
//                            AxesReference.EXTRINSIC, AxesOrder.XZX,
//                            AngleUnit.DEGREES, 90, -90, 0));
//            skystone.setLocationFtcFieldFromTarget(skystoneLocation);
//            RobotLog.ii(TAG, "Skystone=%s", format(skystoneLocation));
//        }
//        else{
//            OpenGLMatrix skystoneLocation = OpenGLMatrix
//                    .translation(0, 0, 4 * mmPerInch)
//                    .multiplied(Orientation.getRotationMatrix(
//                            AxesReference.EXTRINSIC, AxesOrder.XZX,
//                            AngleUnit.DEGREES, 90, 90, 0));
//            skystone.setLocationFtcFieldFromTarget(skystoneLocation);
//            RobotLog.ii(TAG, "Skystone=%s", format(skystoneLocation));
//        }
//
//        // Define position of camera in relation to the robot.
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(mmBotWidth / 2, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZY,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));
//
//        ((VuforiaTrackableDefaultListener) skystone.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//
//        /** Start tracking the data sets we care about. */
//        skystones.activate();
//
////        MatrixF robotXTranslation;
////        MatrixF robotYTranslation;
//
//        boolean buttonPressed = false;
//        double startTime = time;
//        while ((time - startTime) < 10.0) {
//
//            if (buttonPressed == false) {
//                captureFrameToFile();
//                buttonPressed = true;
//            }
//
//            for (VuforiaTrackable trackable : allTrackables) {
//                /**
//                 * getUpdatedRobotLocation() will return null if no new information is available since
//                 * the last time that call was made, or if the trackable is not currently visible.
//                 * getRobotLocation() will return null if the trackable is not currently visible.
//                 */
//
//                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
//
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//            }
//
//            /**
//             * Provide feedback as to where the robot was last located (if we know).
//             */
//            if (lastLocation != null) {
//                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
//                telemetry.addData("Pos", format(lastLocation));
//                telemetry.addData(" ", "");
//                telemetry.addData("X-Value: ", lastLocation.getTranslation().get(0));
//                telemetry.addData(" ", "");
//                telemetry.addData("Y-Value: ", lastLocation.getTranslation().get(1));
//            } else {
//                telemetry.addData("Pos", "Unknown");
//            }
//            telemetry.update();
//        }
//
//        xDifference = lastLocation.getTranslation().get(0);
//        yDifference = lastLocation.getTranslation().get(1);
//
//        xPosition = currentX - xDifference;
//        yPosition = Math.abs(currentY - yDifference) * -1;
//
//        telemetry.addData("X Diference: ", xDifference);
//        telemetry.addData("Y Diference: ", yDifference);
//        telemetry.addData("X Position : ", xPosition);
//        telemetry.addData("Y Position : ", yPosition);
//        telemetry.update();
//        skystones.deactivate();
//
//        sleep(7000);
//    }

    public void setTicksPerInch(double wheelRadius, double encoderTicks){
        double numerator = Math.PI * 2 * wheelRadius;
        double answer = numerator/encoderTicks;

        TICKS_PER_INCH = answer;
    }

    public int getFieldStartPosition(){
        int quadrant = 0;

        if (currentX > 0 && currentY > 0){
            quadrant = 1;
            return quadrant;
        }
        else if (currentX < 0 && currentY > 0){
            quadrant = 2;
            return quadrant;
        }
        else if (currentX < 0 && currentY < 0){
            quadrant = 3;
            return quadrant;
        }
        else if (currentX > 0 && currentY < 0){
            quadrant = 4;
            return quadrant;
        }
        else{
            return quadrant;
        }
    }

    public CoordinatePosition determineSkystone(){
        if (xPosition <= 659.6 && xPosition >= 559.6){
            if (yPosition > -1448.6 && yPosition < -1347.0){
                return redSkystone4;
            }
            else if (yPosition > -1347.0 && yPosition < -1245.4){
                return redSkystone5;
            }
            else if (yPosition > -1245.4 && yPosition < -1143.8){
                return redSkystone6;
            }
            else{
                telemetry.addData("Falied on positive x", " and apparently a bad y");
                telemetry.update();

                return redSkystone1;
            }
        }
        else if(xPosition >= -659.6 && xPosition <= -559.6){
            if (yPosition > -1448.6 && yPosition < -1347.0){
                return blueSkystone4;
            }
            else if (yPosition > -1347.0 && yPosition < -1245.4){
                return blueSkystone5;
            }
            else if (yPosition > -1245.4 && yPosition < -1143.8){
                return blueSkystone6;
            }
            else{
                telemetry.addData("Falied on negative x", " and apparently a bad y");
                telemetry.update();

                return blueSkystone1;
            }
        }
        else{
            telemetry.addData("Failed! ", "I repeat, failed!");
            telemetry.update();

            return blueSkystone1;
        }
    }

    public CoordinatePosition findSecondSkystone(){
        int side = getFieldStartPosition();

        if (side == 2 || side == 3) {
            if (targetSkystone1 == blueSkystone6) {
                return blueSkystone3;
            } else if (targetSkystone1 == blueSkystone5) {
                return blueSkystone2;
            } else {
                return blueSkystone1;
            }
        }
        else{
            if (targetSkystone1 == redSkystone6) {
                return redSkystone3;
            } else if (targetSkystone1 == redSkystone5) {
                return redSkystone2;
            } else {
                return redSkystone1;
            }
        }
    }

    public double determineAngle(CoordinatePosition object){
        double changeX = object.getXCoordinate() - robot.getxCord();
        double changeY = object.getYCoordinate() - robot.getyCord();

        return Math.atan2(changeY, changeX);
    }

    public double determineDistance(CoordinatePosition object){
        double changeX = object.getXCoordinate() - robot.getxCord();
        double changeY = object.getYCoordinate() - robot.getyCord();

        return Math.pow(changeX, 2) + Math.pow(changeY, 2);
    }

    public void redLoadingZone(VuforiaLocalizer.Parameters parameters){
        //findSkystone(parameters);

        gyroDrive(0, 1.0, 26, 5.0);

        gyroTurn(-90, 1.0, 12, 5.0);

        double crrt = robot.frontLeft.getCurrentPosition();

        while (robot.color.alpha() < 22 || robot.color.alpha() > 30){
            gyroDrive(-90, -0.5, 30, 5.0);
        }

        xDiff = (robot.frontLeft.getCurrentPosition()-crrt)/TICKS_PER_INCH;

        gyroDrive(-90, 1.0, 4, 5.0);

        gyroTurn(90, 1.0, 12, 5.0);

        strafe(1.0, 6, 5.0, false);

        robot.intakeLeftServo.setPosition(1);
        robot.intakeRightServo.setPosition(1);

        robot.intakeLeft.setPower(0.85);
        robot.intakeRight.setPower(0.85);

        gyroDrive(90, 1.0, 5, 5.0);

        strafe(1.0, 6.0, 5.0, true);

        if (xDiff <= 13){
            gyroDrive(90, -1.0, 76, 7.0);
        }
        else if(xDiff <= 21){
            gyroDrive(90, -1.0, 84, 7.0);
        }
        else{
            gyroDrive(90, -1.0, 92, 7.0);
        }

        if (robot.touch.isPressed()){
            gyroTurn(179, 1.0, 12, 5.0);
        }
        else {
            gyroTurn(0, 1.0, 12, 5.0);
        }
        robot.intakeLeft.setPower(-1.0);
        robot.intakeRight.setPower(-1.0);
        robot.intakeLeftServo.setPosition(0.6);
        robot.intakeRightServo.setPosition(0.6);

        sleep(1500);

        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);

        gyroTurn(90, 1.0, 12, 5.0);

        if (xDiff <= 13){
            gyroDrive(90, 1.0, 92, 7.0);
        }
        else if(xDiff <= 21){
            gyroDrive(90, 1.0, 100, 7.0);
        }
        else{
            gyroDrive(90, 1.0, 108, 7.0);
        }

        strafe(1.0, 6, 5.0, false);

        robot.intakeLeftServo.setPosition(1);
        robot.intakeRightServo.setPosition(1);

        robot.intakeLeft.setPower(1.0);
        robot.intakeRight.setPower(1.0);

        gyroDrive(90, 1.0, 5, 5.0);

        strafe(1.0, 6.0, 5.0, true);

        if (xDiff <= 13){
            gyroDrive(90, -1.0, 96, 7.0);
        }
        else if(xDiff <= 21){
            gyroDrive(90, -1.0, 104, 7.0);
        }
        else{
            gyroDrive(90, -1.0, 112, 7.0);
        }

        if (robot.touch.isPressed()){
            gyroTurn(179, 1.0, 12, 5.0);
        }
        else {
            gyroTurn(0, 1.0, 12, 5.0);
        }
        robot.intakeLeft.setPower(-1.0);
        robot.intakeRight.setPower(-1.0);
        robot.intakeLeftServo.setPosition(0.6);
        robot.intakeRightServo.setPosition(0.6);

        sleep(1500);

        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);

        gyroTurn(90, 1.0, 12, 5.0);

        gyroDrive(90, 1.0, 50, 5.0);
    }

    public void blueLoadingZone(VuforiaLocalizer.Parameters parameters){
        gyroDrive(0, 1.0, 26, 5.0);

        gyroTurn(-90, 1.0, 12, 5.0);

        double crrt = robot.frontLeft.getCurrentPosition();

        while (robot.color.alpha() < 22 || robot.color.alpha() > 30){
            gyroDrive(-90, 0.5, 30, 5.0);
        }

        xDiff = (robot.frontLeft.getCurrentPosition()-crrt)/TICKS_PER_INCH;

        gyroDrive(-90, -1.0, 4, 5.0);

        strafe(1.0, 6, 5.0, true);

        robot.intakeLeftServo.setPosition(1);
        robot.intakeRightServo.setPosition(1);

        robot.intakeLeft.setPower(1.0);
        robot.intakeRight.setPower(1.0);

        gyroDrive(-90, 1.0, 5, 5.0);

        strafe(1.0, 6.0, 5.0, false);

        if (xDiff <= 13){
            gyroDrive(-90, -1.0, 76, 7.0);
        }
        else if(xDiff <= 21){
            gyroDrive(-90, -1.0, 84, 7.0);
        }
        else{
            gyroDrive(-90, -1.0, 92, 7.0);
        }

        if (robot.touch.isPressed()){
            gyroTurn(-179, 1.0, 12, 5.0);
        }
        else {
            gyroTurn(0, 1.0, 12, 5.0);
        }
        robot.intakeLeft.setPower(-1.0);
        robot.intakeRight.setPower(-1.0);
        robot.intakeLeftServo.setPosition(0.6);
        robot.intakeRightServo.setPosition(0.6);

        sleep(1500);

        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);

        gyroTurn(-90, 1.0, 12, 5.0);

        if (xDiff <= 13){
            gyroDrive(-90, 1.0, 92, 7.0);
        }
        else if(xDiff <= 21){
            gyroDrive(-90, 1.0, 100, 7.0);
        }
        else{
            gyroDrive(-90, 1.0, 108, 7.0);
        }

        strafe(1.0, 6, 5.0, true);

        robot.intakeLeftServo.setPosition(1);
        robot.intakeRightServo.setPosition(1);

        robot.intakeLeft.setPower(1.0);
        robot.intakeRight.setPower(1.0);

        gyroDrive(-90, 1.0, 5, 5.0);

        strafe(1.0, 6.0, 5.0, false);

        if (xDiff <= 13){
            gyroDrive(-90, -1.0, 96, 7.0);
        }
        else if(xDiff <= 21){
            gyroDrive(-90, -1.0, 104, 7.0);
        }
        else{
            gyroDrive(-90, -1.0, 112, 7.0);
        }

        if (robot.touch.isPressed()){
            gyroTurn(-179, 1.0, 12, 5.0);
        }
        else {
            gyroTurn(0, 1.0, 12, 5.0);
        }
        robot.intakeLeft.setPower(-1.0);
        robot.intakeRight.setPower(-1.0);
        robot.intakeLeftServo.setPosition(0.6);
        robot.intakeRightServo.setPosition(0.6);

        sleep(1500);

        robot.intakeLeft.setPower(0);
        robot.intakeRight.setPower(0);

        gyroTurn(90, 1.0, 12, 5.0);

        gyroDrive(90, 1.0, 50, 5.0);
    }

    public void redBuildingZone (VuforiaLocalizer.Parameters parameters){
        gyroDrive(0, 1.0, 30, 5.0);

        gyroTurn(-90, 1.0, 10, 5.0);

        //grab foundation

        strafe(1.0, 30, 5.0, true);

        gyroDrive(-90, 1.0, 30, 5.0);
    }

    public void blueBuildingZone (VuforiaLocalizer.Parameters parameters){
        gyroDrive(0, 1.0, 30, 5.0);

        gyroTurn(-90, 1.0, 10, 5.0);

        //grab foundation

        strafe(1.0, 30, 5.0, false);

        gyroDrive(-90, 1.0, 30, 5.0);
    }

    public void strafe(double speed, double distance, double timeout, boolean direction) {

        distance = distance * TICKS_PER_INCH;

        double rightStart = wheel2.getCurrentPosition(); // top right
        double leftStart = wheel1.getCurrentPosition(); // top left

        double startTime = time;

        if (direction == true) {
            //Strafe left
            while (opModeIsActive() && Math.abs(wheel1.getCurrentPosition() - leftStart) < distance) {

                if (time > startTime + timeout) { // timeout
                    telemetry.addLine("Drive loop timeout.");
                    break;
                }

                wheel1.setPower(speed);
                wheel2.setPower(-speed);
                wheel3.setPower(speed);
                wheel4.setPower(-speed);
            }
            wheel1.setPower(0);
            wheel2.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
        }

        if (direction == false) {
            //Strafe right
            while (opModeIsActive() && Math.abs(wheel2.getCurrentPosition() - rightStart) < distance) {
                if (time > startTime + timeout) { // timeout
                    telemetry.addLine("Drive loop timeout.");
                    break;
                }

                wheel1.setPower(-speed);
                wheel2.setPower(speed);
                wheel3.setPower(-speed);
                wheel4.setPower(speed);
            }
            wheel1.setPower(0);
            wheel2.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
        }
    }

    private void gyroDrive(double targetAngle, double desiredPower, double distance, double timeout) {
        PController controller = new PController(DRIVE_kP);

        distance = distance * TICKS_PER_INCH;

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rightStart = robot.frontRight.getCurrentPosition(); // top right
        double leftStart = robot.frontLeft.getCurrentPosition(); // top left

        double startTime = time;

        double error;

        while (opModeIsActive() && Math.abs(wheel1.getCurrentPosition() - rightStart) < distance && Math.abs(wheel2.getCurrentPosition() - leftStart) < distance) {
            if (time > startTime + timeout) { // timeout
                telemetry.addLine("Drive loop timeout.");
                break;
            }

            Orientation orientation = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double angle = orientation.thirdAngle;
            error = targetAngle - angle;
            if (targetAngle - angle > 180) {
                error = targetAngle - (angle + 360);
            } else if (targetAngle - angle < -180) {
                error = targetAngle - (angle - 360);
            } else {
                error = targetAngle - angle;
            }
            /*if (targetAngle > 90 && angle < -90) {
                error = targetAngle - (angle + 360);
            } else {
                error = targetAngle - angle;
            }*/
            double correction = controller.calculate(error); //controller.calculate(error);
            double rightPower = desiredPower + correction;
            double leftPower  = desiredPower - correction;

            double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) { // clip the power between -1,1 while retaining relative speed percentage
                rightPower = rightPower / max;
                leftPower = leftPower / max;
            }

            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower", leftPower);
            telemetry.addData("Angle: ", angle);
            telemetry.addData("Motor1: ", wheel1.getCurrentPosition());
            telemetry.addData("Motor2: ", wheel2.getCurrentPosition());
            telemetry.addData("rightStart: ",rightStart);
            telemetry.addData("leftStart: ", leftStart);
            telemetry.addData("Tcks: ", TICKS_PER_INCH);
            telemetry.addData("dist: ", distance);
            //telemetry.addData("kP", controller.getkP());
            //telemetry.addData("rightEncoder", bot.wheel1.getCurrentPosition());
            //telemetry.addData("distance", distance);
            //telemetry.addData("Current angle:", orientation.thirdAngle);
            telemetry.update();
            wheel1.setPower(rightPower);
            wheel4.setPower(rightPower);
            wheel2.setPower(leftPower);
            wheel3.setPower(leftPower);
        }

        wheel1.setPower(0);
        wheel4.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
    }

    private void gyroTurn(double targetAngle, double maxSpeed, double distance, double timeout) {
        maxSpeed = Math.abs(maxSpeed); // make sure speed is positive

        // Ethan: if you're getting odd values from this controller, switch the below line to: PController controller = new PController(TURN_kP);
        // Read the notes next to TURN_kP if you're still having trouble
        PIDController controller = new PIDController(TURN_kP, TURN_kI, TURN_kD);
        controller.setOutputClip(1.0);

        distance = distance * TICKS_PER_INCH;

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rightStart = robot.frontRight.getCurrentPosition(); // top right
        double leftStart = robot.frontLeft.getCurrentPosition(); // top left

        double startTime = time;

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double angle = orientation.thirdAngle;
        double lastAngle = angle;
        double error;

        while (Math.abs(wheel1.getCurrentPosition() - rightStart) < distance
                && Math.abs(wheel2.getCurrentPosition() - leftStart) < distance
                && angle != targetAngle
                && opModeIsActive()) {

            if (time > startTime + timeout) { // timeout
                telemetry.addLine("Drive loop timeout.");
                break;
            }

            orientation = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            angle = orientation.thirdAngle;
            if (targetAngle - angle > 180) {
                error = targetAngle - (angle + 360);
            } else if (targetAngle - angle < -180) {
                error = targetAngle - (angle - 360);
            } else {
                error = targetAngle - angle;
            }

            /*if (targetAngle > 90 && angle < -90) {
                error = targetAngle - (angle + 360);
            } else {
                error = targetAngle - angle;
            }*/
            double correction = controller.calculate(error); //controller.calculate(error);
            double rightPower = correction * maxSpeed;
            double leftPower  = -correction * maxSpeed;

            double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) { // clip the power between -1,1 while retaining relative speed percentage
                rightPower = rightPower / max;
                leftPower = leftPower / max;
            }

            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower", leftPower);
            telemetry.update();

            wheel1.setPower(rightPower);
            wheel4.setPower(rightPower);
            wheel2.setPower(leftPower);
            wheel3.setPower(leftPower);
        }

        wheel1.setPower(0);
        wheel4.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
    }
}