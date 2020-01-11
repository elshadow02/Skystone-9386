package org.firstinspires.ftc.teamcode.Path;

public class CircleGenerator {
    public double currentX, currentY;
    public double targetX, targetY;
    public double targetAngle;
    public double A, radiusSquared, circumference;

    public CircleGenerator(double currentX, double currentY, double targetX, double targetY, double targetAngle){
        this.currentX = currentX;
        this.currentY = currentY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
    }

    public double getA() {
        double numerator = Math.sqrt(cube(square(targetX - currentX)+square(targetY - currentY)));
        double denom = Math.cos(90 - targetAngle);

        A =  numerator/denom;
        return A;
    }

    public double getRadiusSquared(){
        double numer = cube(square(targetX - currentX)+square(targetY - currentY));
        double denom = square(Math.cos(90 - targetAngle));

        radiusSquared = numer/denom;

        return radiusSquared;
    }

    public double getCircumference(){
        circumference = 2*Math.PI*getRadiusSquared();

        return circumference;
    }

    private double square(double value){
        return value * value;
    }

    private double cube(double value){
        return value * value * value;
    }
}
