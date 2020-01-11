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

/*
    FTC Team 9386 Elmer and Elsie Robotics Skystone
 */
package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main Hardware Class
 *
 * Made by Ethan. Skystone 2019-20 Season
 * 
 *
 * Motors, Servos, and other hardware devices
 *
 *
 * Rev IMU:                                 "imu"
 *
 * Motor channel:  Front right drive motor: "frontRight"
 * Motor channel:  Front left drive motor:  "frontLeft"
 * Motor channel:  Back left drive motor:   "backLeft"
 * Motor channel:  Back right drive motor:  "backRight"
 */
public class EEHardware {
    /* Public OpMode members. */
    public BNO055IMU imu = null;

    // WHEEL CONFIG: frontRight = FrontRight, frontLeft = FrontLeft
    public DcMotor frontRight      = null;
    public DcMotor frontLeft      = null;
    public DcMotor backLeft      = null;
    public DcMotor backRight      = null;

    public DcMotor intakeLeft     = null;
    public DcMotor intakeRight    = null;
    public DcMotor lift = null;
    public DcMotor arm = null;

    public Servo test = null;
    public Servo intakeLeftServo = null;
    public Servo intakeRightServo = null;
    public Servo claw = null;
    public Servo block = null;

    public double xCord = 0;
    public double yCord = 0;

    public ColorSensor color;
    public DistanceSensor sensorDistance;
    public TouchSensor touch;

    public static final double CORE_HEX_TPR   = 288;  // Core Hex has 4 ppr at base, 72:1 gearbox ratio means 288 ppr at shaft.
    public static final double HD_HEX_TPR     = 2240; // HD Hex Motor has 56  ppr at base, geared at 40:1 creates 2,240 ticks ppr at shaft.
    public static final double NEVEREST20_TPR = 28 * 20;
    public static final double NEVEREST40_TPR = 28 * 40;  // Neverest 40 has 7 ppr at base, 40:1 gearbox ratio means 280 ppr at shaft.
    public static final double NEVEREST60_TPR = 28 * 60;  // Neverest 60 has 7 ppr at base, 60:1 gearbox ratio means 280 ppr at shaft.

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public EEHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//        color = hwMap.get(ColorSensor.class, "sensor");
//        touch = hwMap.get(TouchSensor.class, "touch");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hwMap.get(DistanceSensor.class, "sensor");

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Define and Initialize Motors
        frontRight = hwMap.get(DcMotor.class, "fR");
        frontLeft = hwMap.get(DcMotor.class, "fL");
        backLeft = hwMap.get(DcMotor.class, "bL");
        backRight = hwMap.get(DcMotor.class, "bR");

        intakeLeft = hwMap.get(DcMotor.class, "iL");
        intakeRight = hwMap.get(DcMotor.class, "iR");
//        lift = hwMap.get(DcMotor.class, "lift");
//        arm = hwMap.get(DcMotor.class, "arm");
//
//        test = hwMap.get(Servo.class, "test");
//        claw = hwMap.get(Servo.class, "claw");
//        block = hwMap.get(Servo.class, "block");
//        intakeLeftServo = hwMap.get(Servo.class, "iLS");
//        intakeRightServo = hwMap.get(Servo.class, "iRS");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
//        lift.setPower(0);
//        arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setCords(double xCord, double yCord) {
        this.xCord = xCord;
        this.yCord = yCord;
    }

    public double getxCord(){
        return this.xCord;
    }

    public double getyCord(){
        return this.yCord;
    }

}