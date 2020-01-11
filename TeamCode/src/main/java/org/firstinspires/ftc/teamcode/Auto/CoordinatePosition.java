/*
    FTC Team 9386 Elmer and Elsie Robotics Skystone
 */
package org.firstinspires.ftc.teamcode.Auto;

/**
 * Main Coordinate Position Class.
 *
 * Made by Ethan. Skystone 2019-20 Season
 */
public class CoordinatePosition
{
    public float XCoordinate, YCoordinate, ZCoordinate;

    /* Constructor */
    public CoordinatePosition(){

    }

    public void setXCoordinate(float XCoordinate) {
        this.XCoordinate = XCoordinate;
    }

    public void setYCoordinate(float YCoordinate) {
        this.YCoordinate = YCoordinate;
    }

    public void setZCoordinate(float ZCoordinate) {
        this.ZCoordinate = ZCoordinate;
    }

    public void setXYZ(float XCoordinate, float YCoordinate, float ZCoordinate){
        this.XCoordinate = XCoordinate;
        this.YCoordinate = YCoordinate;
        this.ZCoordinate = ZCoordinate;
    }

    public void setXY(float XCoordinate, float YCoordinate){
        this.XCoordinate = XCoordinate;
        this.YCoordinate = YCoordinate;
    }

    public float getXCoordinate(){
        return this.XCoordinate;
    }

    public float getYCoordinate(){
        return this.YCoordinate;
    }

    public float getZCoordinate(){
        return this.ZCoordinate;
    }
}