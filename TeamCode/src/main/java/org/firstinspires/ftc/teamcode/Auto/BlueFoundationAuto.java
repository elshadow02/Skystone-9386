package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Teleop.EEHardware;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="BlueFoundationAuto", group ="Concept")
public class BlueFoundationAuto extends LinearOpMode {

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

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        wheel1 = bot.frontRight;
        wheel2 = bot.frontLeft;
        wheel3 = bot.backLeft;
        wheel4 = bot.backRight;

        waitForStart();

        drive(1.0, 15, 2.0);

        gyroTurn(180, 1.0, 40, 4.0);

        drive(1.0, -15, 2.0);

        //Engage foundation lock.

        gyroTurn(-90, 1.0, 20, 2.0);

        strafe(1.0, 7, 4.0, true);

        //disengage foundation lock

        strafe(1.0, 23, 5.0, true);

        drive(1.0, 27, 5.0);
    }

    private void strafe(double desiredPower, double distance, double timeout, boolean right) {
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

        while (opModeIsActive() && ((time -startTime)< timeout)) {

            driveControl.setkP(DRIVE_kP);
            driveControl.setkI(DRIVE_kI);
            driveControl.setkD(DRIVE_kD);

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

            if (right == true){
                wheel1.setPower(rightPower * desiredPower);
                wheel4.setPower(-rightPower * desiredPower);
                wheel2.setPower(-leftPower * desiredPower);
                wheel3.setPower(leftPower * desiredPower);
            }
            else {
                wheel1.setPower(-rightPower * desiredPower);
                wheel4.setPower(rightPower * desiredPower);
                wheel2.setPower(leftPower * desiredPower);
                wheel3.setPower(-leftPower * desiredPower);
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

        while (opModeIsActive() && ((time -startTime)< timeout)) {

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
}
