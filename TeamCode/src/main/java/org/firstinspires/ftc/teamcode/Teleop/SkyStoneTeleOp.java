package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleOp")
public class SkyStoneTeleOp extends LinearOpMode {

    @Override
    public void runOpMode () {

        EEHardware robot = new EEHardware();

        // Arm arm = new Arm(hardwareMap);
        // Lift lift = new Lift(hardwareMap);

        double kR = 0.95;
        double clawPos = 0.5;

        int x = 0;

        boolean clawPosBool = false;

        //Define class variables.
        double forward, strafe, rotate;
        double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;
        double joystickAngle, angleChange, speedAngle;

        robot.init(hardwareMap);

        telemetry.addLine("Init Complete");

        waitForStart();

        while (opModeIsActive()) {

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


            if (gamepad1.right_trigger > 0.1) {
                robot.intakeLeft.setPower(gamepad1.right_trigger);
                robot.intakeRight.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intakeLeft.setPower(-gamepad1.left_trigger);
                robot.intakeRight.setPower(-gamepad1.left_trigger);
            } else {
                robot.intakeRight.setPower(0);
                robot.intakeLeft.setPower(0);
            }

            if (robot.up.isPressed() == false && robot.down.isPressed() == false) {
                robot.lift.setPower(-gamepad1.left_stick_y);
                telemetry.addData("Up", "Is Not Pressed");
                telemetry.addData("Down", "Is Not Pressed");
            }
            else if (robot.up.isPressed() == true){
                if ((-gamepad1.left_stick_y) > 0){
                    robot.lift.setPower(0);
                }
                else{
                    robot.lift.setPower(-gamepad1.left_stick_y * 0.6);
                }
                telemetry.addData("Up", "Is Pressed");
            }
            else if (robot.down.isPressed() == true){
                if ((-gamepad1.left_stick_y) < 0){
                    robot.lift.setPower(0);
                }
                else{
                    robot.lift.setPower(-gamepad1.left_stick_y * 0.6);
                }
                telemetry.addData("Down", "Is Pressed");
            }
            else{
                telemetry.addData("ARGH!", "IT BROKE!!");
            }

            robot.arm.setPower(-gamepad2.right_stick_y);

            if (gamepad1.a){
                robot.foundationRight.setPosition(0.15);
            }
            if (gamepad1.x){
                robot.foundationRight.setPosition(1);
            }
            if (gamepad1.b){
                robot.foundationRight.setPosition(0
                );
            }

            if (gamepad1.dpad_up){
                robot.intakeRightServo.setPosition(0.32);
            }

            if (gamepad1.dpad_down){
                robot.intakeRightServo.setPosition(0.85);
            }

            if (gamepad1.dpad_left){
                robot.intakeLeftServo.setPosition(0.25);
            }

            if (gamepad1.dpad_right){
                robot.intakeLeftServo.setPosition(0.55);
            }

            if (gamepad2.b) {
                robot.claw.setPosition(0.15);
            }

            if (gamepad2.x) {
                robot.claw.setPosition(0);
            }

            if (gamepad2.a) {
                clawPos -= 0.01;
            }

            if (gamepad2.y){
                clawPos += 0.01;
            }

            if (gamepad2.dpad_down) {
                robot.claw.setPosition(0);
            }

            //arm.update();

            robot.foundationLeft.setPosition(clawPos);

            telemetry.addData("ClawPos: ", clawPos);
            //telemetry.addData("arm: ", arm.getCurrentArmDegree());
            telemetry.addData("FL Encoder: ", robot.frontLeft.getCurrentPosition());
            telemetry.addData("FR Encoder: ", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL Encoder: ", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR Encoder: ", robot.backRight.getCurrentPosition());
            telemetry.update();

        }
    }

}

