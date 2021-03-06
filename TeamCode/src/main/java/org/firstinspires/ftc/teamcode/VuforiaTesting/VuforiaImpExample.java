package org.firstinspires.ftc.teamcode.VuforiaTesting;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Vuforia Imp Example", group ="Concept")
public class VuforiaImpExample extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * @see #captureFrameToFile()
     */
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    VuforiaImpPlus vuforia;

    WebcamName webcamName;

    float currentX, currentY;

    final float mmPerInch = 25.4f;
    final float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;

    final float mmStonePosition  = (-142/2) + 4f;
    final float mmStoneLength    = 8;

    final float mmFoundationXPosition = 23.75f;
    final float mmFoundationYPosition = 142f/4f;
    final float mmFoundation2XPosition = (23.75f) + (234.95f/25.4f);
    final float mmFoundation2YPosition = (142f/2f) - 977.9f/25.4f;

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

    @Override
    public void runOpMode() throws InterruptedException {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQapn2P/////AAABmRxgWZJT7kKXqyyOQcN4AidYCoBPE/BzcDQASgQ+5iM8wvdkBbzR8qPTDddZSHUp0VsvcAKm8KwIWUElQXmamu9q/iTAzYdJ+aFu/b+2Zyf/+9ZbluiqiSPLptgv/ocKQqzY6nCFoV4qzSFGhH45oRThSBuKmWxrAGJHIo1mnrRdSuyuIOf8JIqo9J9bdqApsVZOSEiuglT7YNQE3DEBAsS9xCLLu8lfn/SvpgzaEy+pBOoehvJOCQ6QabYUz2ZiaaB0CrOLkPjP7OnafVAoo+NZ6vOOqfwRfqEwWUT/YYOoTn8zJLD0+tBdqSZkdVn5sT46CxfZFz1NHfd5RvHzRBcPrI3iB6lXtvCuS8csqLL0";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        vuforia = new VuforiaImpPlus(parameters);

        /**
         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
         * see.
         */
        vuforia.enableConvertFrameToBitmap();

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        telemetry.addData("We are", "starting.");
        telemetry.update();

        sleep(5000);

        vuMarkInit(5.0, currentX, currentY, parameters);

        telemetry.addData("We are", "closing.");
        telemetry.update();

        sleep(2000);

        vuforia.close();

        telemetry.clearAll();
        telemetry.update();

        telemetry.addData("We are", "done");
        telemetry.update();

        //sleep(5000);
        //waitForStart();

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

    }

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
    public void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }

    public void vuMarkInit(double maxTime, float xPosition, float yPosition, VuforiaLocalizer.Parameters parameters) {

        /*
        Define VuMarks.
        Rear/Back is in the Building Zone.
        Front is in the Loading Zone.
         */
        VuforiaTrackables skystone = vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable bridgeBlueBack = skystone.get(1);
        bridgeBlueBack.setName("bridgeBlueBack");

        VuforiaTrackable bridgeRedBack = skystone.get(2);
        bridgeRedBack.setName("bridgeRedBack");

        VuforiaTrackable bridgeRedFront = skystone.get(3);
        bridgeRedBack.setName("bridgeRedFront");

        VuforiaTrackable bridgeBlueFront = skystone.get(4);
        bridgeRedBack.setName("bridgeBlueFront");

        VuforiaTrackable RedPerimeterBack = skystone.get(5);
        bridgeRedBack.setName("RedPerimeterBack");

        VuforiaTrackable RedPerimeterFront = skystone.get(6);
        bridgeRedBack.setName("RedPerimeterFront");

        VuforiaTrackable FrontPerimeterRed = skystone.get(7);
        bridgeRedBack.setName("FrontPerimeterRed");

        VuforiaTrackable FrontPerimeterBlue = skystone.get(8);
        bridgeRedBack.setName("FrontPerimeterBlue");

        VuforiaTrackable BluePerimeterFront = skystone.get(9);
        bridgeRedBack.setName("BluePerimeterFront");

        VuforiaTrackable BluePerimeterBack = skystone.get(10);
        bridgeRedBack.setName("BluePerimeterBack");

        VuforiaTrackable RearPerimeterBlue = skystone.get(11);
        bridgeRedBack.setName("RearPerimeterBlue");

        VuforiaTrackable RearPerimeterRed = skystone.get(12);
        bridgeRedBack.setName("RearPerimeterRed");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(skystone);

        //Convert field measurements to mm because Skystone XML file data uses mm.
        // the FTC field is ~11'10" (142") center-to-center of the glass panels
        final float vumarkHeight = 5.75f * mmPerInch;
        final float vumarkDistanceFromWallCenter = 35 * mmPerInch;
        final float bridgeVumarkXDistanceFromOrigin = 24 * mmPerInch;
        final float bridgeVumarkYDistanceFromOrigin = 9 * mmPerInch;

        final float xOrigin = 0.0f;
        final float yOrigin = 0.0f;
        final float zOrigin = 0.0f;

        final float X_MAX = mmFTCFieldWidth / 2;
        final float Y_MAX = mmFTCFieldWidth / 2;
        final float Z_MAX = mmFTCFieldWidth / 2;

        final float X_MIN = -mmFTCFieldWidth / 2;
        final float Y_MIN = -mmFTCFieldWidth / 2;
        final float Z_MIN = 0.0f;


        //Define Rear Perimeter Target 1 position on field.
        OpenGLMatrix RearPerimeterBlueLocation = OpenGLMatrix
                .translation(-vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RearPerimeterBlue.setLocationFtcFieldFromTarget(RearPerimeterBlueLocation);
        RobotLog.ii(TAG, "Rear Perimeter Blue=%s", format(RearPerimeterBlueLocation));

        //Define Rear Perimeter Target 2 position on field.
        OpenGLMatrix RearPerimeterRedLocation = OpenGLMatrix
                .translation(vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RearPerimeterRed.setLocationFtcFieldFromTarget(RearPerimeterRedLocation);
        RobotLog.ii(TAG, "Rear Perimeter Red=%s", format(RearPerimeterRedLocation));

        //Define Front Perimeter Target 2 position on field.
        OpenGLMatrix FrontPerimeterBlueLocation = OpenGLMatrix
                .translation(-vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        FrontPerimeterBlue.setLocationFtcFieldFromTarget(FrontPerimeterBlueLocation);
        RobotLog.ii(TAG, "Front Perimeter Blue=%s", format(FrontPerimeterBlueLocation));

        //Define Front Perimeter Target 1 position on field.
        OpenGLMatrix FrontPerimeterRedLocation = OpenGLMatrix
                .translation(vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        FrontPerimeterRed.setLocationFtcFieldFromTarget(FrontPerimeterRedLocation);
        RobotLog.ii(TAG, "Front Perimeter Red=%s", format(FrontPerimeterRedLocation));

        //Define Red Perimeter Target 1 position on field.
        OpenGLMatrix RedPerimeterBackLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        RedPerimeterBack.setLocationFtcFieldFromTarget(RedPerimeterBackLocation);
        RobotLog.ii(TAG, "Red Perimeter Back=%s", format(RedPerimeterBackLocation));

        //Define Red Perimeter Target 2 position on field.
        OpenGLMatrix RedPerimeterFrontLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        RedPerimeterFront.setLocationFtcFieldFromTarget(RedPerimeterFrontLocation);
        RobotLog.ii(TAG, "Red Perimeter Front=%s", format(RedPerimeterFrontLocation));

        //Define Blue Perimeter Target 2 position on field.
        OpenGLMatrix BluePerimeterBackLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        BluePerimeterBack.setLocationFtcFieldFromTarget(BluePerimeterBackLocation);
        RobotLog.ii(TAG, "Blue Perimeter Back=%s", format(BluePerimeterBackLocation));

        //Define Blue Perimeter Target 1 position on field.
        OpenGLMatrix BluePerimeterFrontLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        BluePerimeterFront.setLocationFtcFieldFromTarget(BluePerimeterFrontLocation);
        RobotLog.ii(TAG, "Blue Perimeter Front=%s", format(BluePerimeterFrontLocation));

        //Define Bridge Blue Back position on field.
        OpenGLMatrix BridgeBlueBackLocation = OpenGLMatrix
                .translation(-bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        bridgeBlueBack.setLocationFtcFieldFromTarget(BridgeBlueBackLocation);
        RobotLog.ii(TAG, "Bridge Blue Back=%s", format(BridgeBlueBackLocation));

        //Define Red Bridge Back position on field.
        OpenGLMatrix BridgeRedBackLocation = OpenGLMatrix
                .translation(bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        bridgeRedBack.setLocationFtcFieldFromTarget(BridgeRedBackLocation);
        RobotLog.ii(TAG, "Red Bridge Back=%s", format(BridgeRedBackLocation));

        //Define Bridge Blue Front position on field.
        OpenGLMatrix BridgeBlueFrontLocation = OpenGLMatrix
                .translation(-bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        bridgeBlueFront.setLocationFtcFieldFromTarget(BridgeBlueFrontLocation);
        RobotLog.ii(TAG, "Blue Bridge Front=%s", format(BridgeBlueFrontLocation));

        //Define Red Bridge Front position on field.
        OpenGLMatrix BridgeRedFrontLocation = OpenGLMatrix
                .translation(bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        bridgeRedFront.setLocationFtcFieldFromTarget(BridgeRedFrontLocation);
        RobotLog.ii(TAG, "Red Bridge Front=%s", format(BridgeRedFrontLocation));


        // Define position of camera in relation to the robot.
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth, 6f*25.4f, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 45, 0));
        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));

        /**
         * Let the trackable listeners we care about know where the camera is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) RearPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RearPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) FrontPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) FrontPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) BluePerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) BluePerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RedPerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RedPerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeBlueBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeBlueFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeRedBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeRedFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);

        /** Start tracking the data sets we care about. */
        skystone.activate();

//        MatrixF robotXTranslation;
//        MatrixF robotYTranslation;

        boolean buttonPressed = false;
        double startTime = time;
        while ((time - startTime) < maxTime) {

            if (buttonPressed == true) {
                captureFrameToFile();
                buttonPressed = false;
            }

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

//            robotXTranslation = lastLocation.getTranslation().get(0);
//            robotYTranslation = lastLocation.getTranslation().get(1);

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
                telemetry.addData(" ", "");
                telemetry.addData("X-Value: ", lastLocation.getTranslation().get(0));
                telemetry.addData(" ", "");
                telemetry.addData("Y-Value: ", lastLocation.getTranslation().get(1));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }

//        currentX = lastLocation.getTranslation().get(0);
//        currentY = lastLocation.getTranslation().get(1);

        //Debugging
        currentX = 1;
        currentY = 1;

        skystone.deactivate();
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
