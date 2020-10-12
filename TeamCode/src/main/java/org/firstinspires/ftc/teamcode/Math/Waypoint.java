package org.firstinspires.ftc.teamcode.Math;

public class Waypoint {
    public enum Type {BASE, CURVE_START, CURVE_END}

    private double x;
    private double y;
    private Type type;
    private double time;
    public Waypoint(){
        x=0;
        y=0;
        type=Type.BASE;
        time = -1;
    }
    public Waypoint(double x, double y){
        this.x=x;
        this.y=y;
        type=Type.BASE;
        time = -1;
    }
    public Waypoint(double x, double y, Type type){
        this.x=x;
        this.y=y;
        this.type=type;
        this.time=-1;
    }
    public Waypoint(double x, double y, Type type, double time){
        this.x=x;
        this.y=y;
        this.type=type;
        this.time=time;
    }
    public Waypoint(Waypoint waypoint, Type type){
        this.x=waypoint.getX();
        this.y=waypoint.getY();
        this.type=type;
        this.time=waypoint.getTime();
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public Type getType(){
        return type;
    }
    public double getTime(){
        return time;
    }
    public void setTime(double t){
        time = t;
    }
    public double angleTo(Waypoint waypointFinal){
        return Math.atan2(waypointFinal.getY()-this.getY(), waypointFinal.getX()-this.getX());
    }
    public double angleFrom(Waypoint waypointStart){
        return Math.atan2(this.getY()-waypointStart.getY(), this.getX()-waypointStart.getX());
    }
    public double distanceFrom(Waypoint waypointStart){
        return Math.hypot(waypointStart.getX()-this.getX(), waypointStart.getY()-this.getY());
    }
}