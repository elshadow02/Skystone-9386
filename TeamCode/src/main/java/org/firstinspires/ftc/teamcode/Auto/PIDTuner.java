package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;

//import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

@TeleOp(name="PIDTuner")
@Config
    public class PIDTuner extends OpMode {

        public static int travel = 90;

        public static double maxPower = 1;

        public static double kp = 0.00092;
        public static double ki = 0.0;
        public static double kd = 1.0;

        public DcMotor motor      = null;

        public PIDController pid = new PIDController(kp, ki, kd);

        public static double gearRatio = 2;

        public double ticksPerAngle = 10;

        public static int start = 1;

        //public double kp, ki, kd;
        double startMotorPos;
        public int x = 0;

        public int loopCount = 0;


        @Override // @Override tells the computer we intend to override OpMode's method init()
        public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            motor = hardwareMap.get(DcMotor.class, "arm");
            //ticksPerAngle = 10;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            startMotorPos = motor.getCurrentPosition();


            //kp = 0.1;
            //ki = 0.0;
            //kd = 0.0;
        }

        @Override
        public void loop() {
            if(start == 2) {
                motor.setTargetPosition(900);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);

                while (motor.isBusy()) {

                    // Display it for the driver.
                    telemetry.addLine("The current position: " + motor.getCurrentPosition());
                    telemetry.update();
                }

//                motor.setPower(0);

                motor.setTargetPosition(0);
            }

            pid.setkP(kp);
            pid.setkI(ki);
            pid.setkD(kd);
            x = 0;
            if (gamepad1.a){
                if (x == 0) {
                    travel += 1;
                    x = 1;
                }
            }

            if (gamepad1.b){
                if (x == 0) {
                    travel += 10;
                    x = 1;
                }
            }

            //travel = 1;

            telemetry.addData("Distance to travel: ", travel);
            telemetry.addData("Encoder Value: ", motor.getCurrentPosition());
            telemetry.update();

            if (start == 1){
                setPosition(travel);
            }
        }

        public void setPosition(int angle){
            angle *= ticksPerAngle;

            double output;

            double error;

            //motor.setPower(0.2);

//            do {
//                pid.setkP(kp);
//                pid.setkI(ki);
//                pid.setkP(kd);

                error = angle - (motor.getCurrentPosition() - startMotorPos);

                //output = error * kp;
                output = pid.calculate(error);

                motor.setPower(output);

                telemetry.addData("Encoder target: ", angle);
                telemetry.addData("Error: ", error);
                telemetry.addData("Output: ", output);
                telemetry.addData("kp: ", kp);
                telemetry.addData("ki: ", ki);
                telemetry.addData("kd: ", kd);
                telemetry.addData("loop count: ", loopCount);
                telemetry.update();
                loopCount += 1;
//                Thread.yield();
//            }
//            while (true);


//            motor.setPower(0);
//
//            start = 0;
        }

    }

