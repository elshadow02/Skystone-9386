package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Typical Mecanum Drive")
public class TypicalMecanumDrive extends OpMode {

    private EEHardware robot = new EEHardware();

    double kR = 0.95;

    public DcMotor intakeLeft     = null;
    public DcMotor intakeRight    = null;

    //Define class variables.
    private double forward, strafe, rotate;
    private double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        intakeLeft = hardwareMap.get(DcMotor.class, "iL");
        intakeRight = hardwareMap.get(DcMotor.class, "iR");
        //robot.init(hardwareMap);

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        rotate = gamepad1.right_stick_x;
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;

        rotate *= kR;

        frontLeftSpeed = forward + rotate + strafe;
        frontRightSpeed = forward - rotate - strafe;
        backLeftSpeed = forward + rotate - strafe;
        backRightSpeed = forward - rotate + strafe;

        double max = Math.abs(frontLeftSpeed);
        if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
        if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
        if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);

        if (max > 1) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        robot.frontRight.setPower(frontRightSpeed);
        robot.frontLeft.setPower(frontLeftSpeed);
        robot.backLeft.setPower(backLeftSpeed);
        robot.backRight.setPower(backRightSpeed);

        if (gamepad1.left_trigger > 0.1) {
            intakeLeft.setPower(-gamepad1.left_trigger);
            intakeRight.setPower(-gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger > 0.1) {
            intakeLeft.setPower(gamepad1.right_trigger);
            intakeRight.setPower(gamepad1.right_trigger);
        }
        else{
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

}
