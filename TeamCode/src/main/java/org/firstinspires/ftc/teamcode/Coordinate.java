package org.firstinspires.ftc.teamcode;

public class Coordinate {
    private double xAxis;
    private double  yAxis;
    private double heading;
    public Coordinate(double x, double y, double headingDegrees){
        xAxis = x;
        yAxis = y;
        heading = Math.toRadians(headingDegrees);
    }
    public double getX(){
        return xAxis;
    }
    public double getY(){
        return yAxis;
    }
    public double getHeading(){
        return heading;
    }
}


