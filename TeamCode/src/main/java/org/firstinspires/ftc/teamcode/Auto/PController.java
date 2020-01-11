package org.firstinspires.ftc.teamcode.Auto;

public class PController extends PIDController{

    public PController (double kP) {
        super(kP, 0, 0);
    }
}