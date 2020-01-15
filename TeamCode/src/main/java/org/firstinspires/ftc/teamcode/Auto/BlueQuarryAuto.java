package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.teamcode.Auto.CVExample;
import org.firstinspires.ftc.teamcode.Teleop.EEHardware;
import org.firstinspires.ftc.teamcode.VuforiaTesting.VuforiaImpPlus;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="BlueQuarryAuto", group ="Concept")
public class BlueQuarryAuto extends LinearOpMode {

    EEHardware bot = new EEHardware();

    private DcMotor wheel1;
    private DcMotor wheel2;
    private DcMotor wheel3;
    private DcMotor wheel4;

    public static double DRIVE_kP = 0.001;
    public static double DRIVE_kI = 0.0000007;
    public static double DRIVE_kD = 1;

    public static double TURN_kP = 0.05;  // increase this number to increase responsiveness. decrease this number to decrease oscillation
    public static double TURN_kI = 0.01;  // increase this number to decrease steady state error (controller stops despite error not equalling 0)
    public static double TURN_kD = 0.025;

    double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    double TICKS_PER_ROTATION = 537.6;
    double TICKS_PER_INCH      = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    private ElapsedTime runtime = new ElapsedTime();

    public int SkystonePos = 1; //Position of Skystone: 1 = Left; 2 = middle; 3 = right.

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        wheel1 = bot.frontRight;
        wheel2 = bot.frontLeft;
        wheel3 = bot.backLeft;
        wheel4 = bot.backRight;

        StageSwitchingPipeline pipeline = new StageSwitchingPipeline();

        //int cameraMonitorViewIdSkystone = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewIdSkystone", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        phoneCam.openCameraDevice();//open camera
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        phoneCam.setPipeline(pipeline);
        // phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        telemetry.addData("We are", "5");
        telemetry.update();
        //width, height
        //width = height in this case, because camera is in portrait mode.

        telemetry.clear();

        runtime.reset();
        while (!opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
        waitForStart();

        if (valLeft == 0 && valMid > 0 && valRight > 0){
            SkystonePos = 1;
        }
        else if (valLeft > 0 && valMid == 0 && valRight > 0){
            SkystonePos = 2;
        }
        else if (valLeft > 0 && valMid > 0 && valRight == 0){
            SkystonePos = 3;
        }
        else{
            telemetry.addData("Something went ", "Super Duper wrong.");
            telemetry.update();
        }

        telemetry.addData("Skystone Pos ", SkystonePos);
        telemetry.update();

        //sleep(2000);
        double travel = 0;

        if (SkystonePos == 1){
            travel = 14.25;
        }
        else if (SkystonePos == 2){
            travel = 23.25;
        }
        else{
            travel = 31.75;
        }

        strafe(0.7, travel, 5.0, true);

        bot.intakeLeft.setPower(1);
        bot.intakeRight.setPower(1);

        drive(0.4, 38.5, 6.0);

        sleep (1000);

        bot.intakeLeft.setPower(0);
        bot.intakeRight.setPower(0);

        drive(1.0, -6, 2.0);

        gyroTurn(90, 1.0, 20, 3.0);

        if (SkystonePos == 1){
            drive(1.0, 42, 5.0);
        }
        else if (SkystonePos == 2){
            drive(1.0, 50, 5.0);
        }
        else{
            drive(1.0, 58, 5.0);
        }

        bot.intakeLeft.setPower(-1);
        bot.intakeRight.setPower(-1);

        sleep(1500);

        bot.intakeLeft.setPower(0);
        bot.intakeRight.setPower(0);

        if (SkystonePos == 1){
            drive(1.0, -59, 5.0);
        }
        else if (SkystonePos == 2){
            drive(1.0, -65, 5.0);
        }
        else{
            drive(1.0, -61, 5.0);
        }

        strafe(1.0, 8, 2.0,false);

        if(SkystonePos == 3){
            gyroTurn(-20, 1.0, 29, 3.0);
        }
        else {
            gyroTurn(0, 1.0, 21.5, 3.0);
        }

        bot.intakeLeft.setPower(1);
        bot.intakeRight.setPower(1);

        if(SkystonePos == 3){
            drive (0.5, 18.0, 5.0);
        }
        else {
            drive (0.5, 14.0, 5.0);
        }

        sleep(1500);

        bot.intakeLeft.setPower(0);
        bot.intakeRight.setPower(0);

        drive(1.0, -6.0, 2.0);

        strafe(1.0, 8, 2.0, false);

        if(SkystonePos == 3){
            gyroTurn(90, 1.0, 24, 3.0);
        }
        else {
            gyroTurn(90, 1.0, 20, 3.0);
        }

        if (SkystonePos == 1){
            drive(1.0, 69, 5.0);
        }
        else if (SkystonePos == 2){
            drive(1.0, 77, 5.0);
        }
        else{
            drive(1.0, 81, 5.0);
        }

        bot.intakeLeft.setPower(-1);
        bot.intakeRight.setPower(-1);

        sleep(1500);

        bot.intakeLeft.setPower(0);
        bot.intakeRight.setPower(0);

        drive(1.0, -12, 2.0);
    }

    private void strafe(double desiredPower, double distance, double timeout, boolean right) {
        //PIDController controller = new PIDController(DRIVE_kP, 0, 0);
        //PIDController driveControl = new PIDController(DRIVE_kP, DRIVE_kI, DRIVE_kD);

        distance = distance * TICKS_PER_INCH;

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rightStart = bot.frontRight.getCurrentPosition(); // top right
        double leftStart = bot.frontLeft.getCurrentPosition(); // top left

        double startTime = time;

        double angleEerror, driveError;
        double rightError, leftError;

        while (opModeIsActive() && ((time -startTime)< timeout) && Math.abs(bot.frontRight.getCurrentPosition() - rightStart) <= distance && Math.abs(bot.frontLeft.getCurrentPosition() - leftStart) <= distance) {

//            driveControl.setkP(DRIVE_kP);
//            driveControl.setkI(DRIVE_kI);
//            driveControl.setkD(DRIVE_kD);

            rightError = distance - (wheel1.getCurrentPosition() - rightStart);
            leftError = distance - (wheel2.getCurrentPosition() - leftStart);

            driveError = (rightError + leftError) / 2;

            // telemetry.addData("error", angleEerror);
            //telemetry.addData("correction", correction);
            telemetry.addData("rightEncoder: ", bot.frontRight.getCurrentPosition());
            telemetry.addData("leftEncoder: ", bot.frontLeft.getCurrentPosition());
            telemetry.addData("distance: ", distance);
            telemetry.addData("Right Error: ", rightError);
            telemetry.addData("Left Error: ", leftError);
            //telemetry.addData("Current angle:", orientation.thirdAngle);
            telemetry.update();

            if (right == false){
                wheel1.setPower(desiredPower);
                wheel4.setPower(-desiredPower);
                wheel2.setPower(-desiredPower);
                wheel3.setPower(desiredPower);
            }
            else {
                wheel1.setPower(-desiredPower);
                wheel4.setPower(desiredPower);
                wheel2.setPower(desiredPower);
                wheel3.setPower(-desiredPower);
            }
        }
        wheel1.setPower(0);
        wheel4.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);

        telemetry.addData("Finished with the loop: ", "right?");
        //telemetry.addData("Current angle:", orientation.thirdAngle);
        telemetry.update();
    }

    private void drive(double desiredPower, double distance, double timeout) {
        //PIDController controller = new PIDController(DRIVE_kP, 0, 0);
        PIDController driveControl = new PIDController(DRIVE_kP, DRIVE_kI, DRIVE_kD);
        distance = distance * TICKS_PER_INCH;

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rightStart = bot.frontRight.getCurrentPosition(); // top right
        double leftStart = bot.frontLeft.getCurrentPosition(); // top left

        double startTime = time;

        double angleEerror, driveError;
        double rightError, leftError;

        do {

            driveControl.setkP(DRIVE_kP);
            driveControl.setkI(DRIVE_kI);
            driveControl.setkD(DRIVE_kD);

//            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
////            double angle = orientation.thirdAngle;
////            angleError = targetAngle - angle;
////            if (targetAngle - angle > 180) {
////                angleError = targetAngle - (angle + 360);
////            } else if (targetAngle - angle < -180) {
////                angleError = targetAngle - (angle - 360);
////            } else {
////                angleError = targetAngle - angle;
////            }

            rightError = distance - (wheel1.getCurrentPosition() - rightStart);
            leftError = distance - (wheel2.getCurrentPosition() - leftStart);

            driveError = (rightError + leftError) / 2;

            //double correction = controller.calculate(angleEerror); //controller.calculate(error);

            double errorPower = driveControl.calculate(driveError);

            double rightPower = errorPower;
            double leftPower = errorPower;

            double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) { // clip the power between -1,1 while retaining relative speed percentage
                rightPower = rightPower / max;
                leftPower = leftPower / max;
            }

            // telemetry.addData("error", angleEerror);
            //telemetry.addData("correction", correction);
            telemetry.addData("rightPower: ", rightPower);
            telemetry.addData("leftPower: ", leftPower);
            telemetry.addData("driveError: ", driveError);
            telemetry.addData("Error power: ", errorPower);
            telemetry.addData("rightEncoder: ", bot.frontRight.getCurrentPosition());
            telemetry.addData("leftEncoder: ", bot.frontLeft.getCurrentPosition());
            telemetry.addData("distance: ", distance);
            telemetry.addData("Right Error: ", rightError);
            telemetry.addData("Left Error: ", leftError);
            //telemetry.addData("Current angle:", orientation.thirdAngle);
            telemetry.update();
            wheel1.setPower(rightPower * desiredPower);
            wheel4.setPower(rightPower * desiredPower);
            wheel2.setPower(leftPower * desiredPower);
            wheel3.setPower(leftPower * desiredPower);
        }
        while (opModeIsActive() && ((time -startTime)< timeout) && Math.abs(bot.frontRight.getPower()) > 0.15 && Math.abs(bot.frontLeft.getPower()) > 0.15);
        wheel1.setPower(0);
        wheel4.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);

        telemetry.addData("Finished with the loop: ", "right?");
        //telemetry.addData("Current angle:", orientation.thirdAngle);
        telemetry.update();
    }

    private void gyroTurn(double targetAngle, double maxSpeed, double distance, double timeout) {
        maxSpeed = Math.abs(maxSpeed); // make sure speed is positive

        // Ethan: if you're getting odd values from this controller, switch the below line to: PController controller = new PController(TURN_kP);
        // Read the notes next to TURN_kP if you're still having trouble
        PIDController controller = new PIDController(TURN_kP, TURN_kI, TURN_kD);

        distance = distance * TICKS_PER_INCH;

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rightStart = bot.frontRight.getCurrentPosition(); // top right
        double leftStart = bot.frontLeft.getCurrentPosition(); // top left

        double startTime = time;

        Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double angle = orientation.thirdAngle;
        double lastAngle = angle;
        double error;
        int count = 0;

        while (Math.abs(wheel1.getCurrentPosition() - rightStart) < distance
                && Math.abs(wheel2.getCurrentPosition() - leftStart) < distance
                && angle != targetAngle
                && opModeIsActive()) {

            if (time > startTime + timeout) { // timeout
                telemetry.addLine("Drive loop timeout.");
                break;
            }

            orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            angle = orientation.thirdAngle;
            if (targetAngle - angle > 180) {
                error = targetAngle - (angle + 360);
            } else if (targetAngle - angle < -180) {
                error = targetAngle - (angle - 360);
            } else {
                error = targetAngle - angle;
            }

            double correction = controller.calculate(error); //controller.calculate(error);
            double rightPower = correction * maxSpeed;
            double leftPower  = -correction * maxSpeed;

            double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) { // clip the power between -1,1 while retaining relative speed percentage
                rightPower = rightPower / max;
                leftPower = leftPower / max;
            }
            count += 1;

            telemetry.addData("loop count", count);
            telemetry.addData("error", error);
            telemetry.addData("Angle", angle);
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

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
