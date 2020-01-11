package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

@TeleOp(name="MotorTest")
    public class MotorTest extends OpMode {

        public static int travel = 0;

        public static double kp = 0.1;
        public static double ki = 0.0;
        public static double kd = 0.0;

        public DcMotor motor, motor1, motor2, motor3      = null;
        public Servo servo = null;

        public PIDController pid = new PIDController(0.1, 0, 0);

        public static double gearRatio = 1;

        public double ticksPerInch = 1;

        //public double kp, ki, kd;
        public int x = 0;


        @Override // @Override tells the computer we intend to override OpMode's method init()
        public void init() {
            motor = hardwareMap.get(DcMotor.class, "motor");
            motor1 = hardwareMap.get(DcMotor.class, "motor1");
            motor2 = hardwareMap.get(DcMotor.class, "motor2");
            motor3 = hardwareMap.get(DcMotor.class, "motor3");
            servo = hardwareMap.get(Servo.class, "servo");
            ticksPerInch = gearRatio / motor.getMotorType().getTicksPerRev();
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            kp = 0.1;
            ki = 0.0;
            kd = 0.0;
        }

        @Override
        public void loop() {
            motor.setPower(-gamepad1.left_stick_y);
            motor1.setPower(-gamepad1.left_stick_y);
            motor2.setPower(-gamepad1.left_stick_y);
            motor3.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Motor 1 Power: ", motor.getPower());
            telemetry.addData("Encoder 1 Value: ", motor.getCurrentPosition());
            telemetry.addData("Motor 2 Power: ", motor1.getPower());
            telemetry.addData("Encoder 2 Value: ", motor1.getCurrentPosition());
            telemetry.addData("Motor 3 Power: ", motor2.getPower());
            telemetry.addData("Encoder 3 Value: ", motor2.getCurrentPosition());
            telemetry.addData("Motor 4 Power: ", motor3.getPower());
            telemetry.addData("Encoder 4 Value: ", motor3.getCurrentPosition());
            telemetry.update();
        }

        public void setPosition(int inches){
            inches *= ticksPerInch;

            double output = 1;

            double error;

            do {
                pid.setkP(kp);
                pid.setkI(ki);
                pid.setkP(kd);

                error = inches - motor.getCurrentPosition();

                output = pid.calculate(error);

                motor.setPower(output);

                telemetry.addData("Encoder target: ", inches);
                telemetry.addData("Error: ", error);
                telemetry.addData("Output: ", output);
                telemetry.addData("kp: ", kp);
                telemetry.addData("ki: ", ki);
                telemetry.addData("kd: ", kd);
                telemetry.update();
            }
            while (motor.isBusy());


            motor.setPower(0);
        }

    }

