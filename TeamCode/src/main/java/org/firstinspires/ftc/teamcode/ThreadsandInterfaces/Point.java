package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

public class Point implements Execution{
    private double x;
    private double y;
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public void run(){

    }
}
