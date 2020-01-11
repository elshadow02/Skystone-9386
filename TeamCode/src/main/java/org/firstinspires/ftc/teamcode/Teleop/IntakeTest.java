package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Typical intake")
public class IntakeTest extends OpMode {

    //private EEHardware robot = new EEHardware();

    public DcMotor intakeLeft     = null;
    public DcMotor intakeRight    = null;

    double kR = 0.95;

    //Define class variables.
    private double forward, strafe, rotate;
    private double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {

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
        intakeLeft.setPower(-gamepad1.left_stick_y);
        intakeRight.setPower(-gamepad1.left_stick_y);
    }

}
