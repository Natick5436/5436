package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry extends Thread{
    //wheelDiameter in centimeters.
    double wheelDiameter;
    //wheel Circumference in meters for ease of odometry use.
    double wheelCircum;
    //ticks per full revolution of the motor axle
    double ticksPerMotorRev;
    double motorGearRatio;
    //ticks per full revolution of the wheel
    double ticksPer;
    //distance in meters between the two encoded wheels
    double LENGTH;

    public double odometryX;
    public double odometryY;
    public double odometryAngle;
    double lastEncoderR, lastEncoderL, lastEncoderMiddle;
    double currentEncoderL, currentEncoderR, currentEncoderMiddle;

    private DcMotor left;
    private double lDirection;
    private DcMotor right;
    private double rDirection;
    private DcMotor middle;
    private double mDirection;

    LinearOpMode ln;
    public Odometry(LinearOpMode linear, DcMotor l, DcMotor r, DcMotor m, double wheelDiameter, double ticksPerMotorRev, double motorGearRatio, double LENGTH, double initialX, double initialY, double initialAngle){
        ln = linear;
        left = l;
        right = r;
        middle = m;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircum = wheelDiameter*Math.PI/100.0;
        this.ticksPerMotorRev = ticksPerMotorRev;
        this.motorGearRatio = motorGearRatio;
        this.ticksPer = motorGearRatio*ticksPerMotorRev;
        this.LENGTH = LENGTH;
        odometryX = initialX;
        odometryY = initialY;
        odometryAngle = initialAngle;
        lDirection = 1;
        rDirection = 1;
        mDirection = 1;
    }

    public void setLeftDirection(DcMotor.Direction direction){
        lDirection = (direction == DcMotorSimple.Direction.REVERSE)? -1 : 1;
    }
    public void setRightDirection(DcMotor.Direction direction){
        rDirection = (direction == DcMotorSimple.Direction.REVERSE)? -1 : 1;
    }
    public void setMiddleDirection(DcMotor.Direction direction){
        mDirection = (direction == DcMotorSimple.Direction.REVERSE)? -1 : 1;
    }

    public double getX(){
        return odometryX;
    }
    public double getY(){
        return odometryY;
    }
    public double getAngle(){
        return odometryAngle;
    }
    public void resetAngle(double overrideAngle){
        odometryAngle = overrideAngle;
    }

    public void run(){
        lastEncoderL = lDirection*left.getCurrentPosition();
        lastEncoderR = rDirection*right.getCurrentPosition();
        lastEncoderMiddle = mDirection*middle.getCurrentPosition();
        while (ln.opModeIsActive()){
            currentEncoderL = lDirection*left.getCurrentPosition();
            currentEncoderR = rDirection*right.getCurrentPosition();
            currentEncoderMiddle = mDirection*middle.getCurrentPosition();
            double deltaTickL = currentEncoderL - lastEncoderL;
            double deltaTickR = currentEncoderR - lastEncoderR;
            double deltaTickMiddle = currentEncoderMiddle - lastEncoderMiddle;
            double dL = wheelCircum * deltaTickL / ticksPer;
            double dR = wheelCircum * deltaTickR / ticksPer;
            double dMiddle = wheelCircum * deltaTickMiddle / ticksPer;
            double dM = (dL + dR) / 2;
            double dAngle = (dR - dL) / LENGTH;

            if(dAngle != 0) {
                odometryX = odometryX + (dM * Math.sin(dAngle) * Math.cos(odometryAngle + (dAngle / 2)) /
                        (dAngle * Math.cos(dAngle / 2))) + (dMiddle*Math.cos((odometryAngle + (dAngle/2))-Math.PI/2));
                odometryY = odometryY + (dM * Math.sin(dAngle) * Math.sin(odometryAngle + (dAngle / 2)) /
                        (dAngle * Math.cos(dAngle / 2))) + (dMiddle*Math.sin((odometryAngle + (dAngle/2))-Math.PI/2));
                odometryAngle = odometryAngle + dAngle;
            }else{
                odometryX = odometryX + (dM * Math.cos(odometryAngle)) + (dMiddle*Math.cos((odometryAngle)-Math.PI/2));
                odometryY = odometryY + (dM * Math.sin(odometryAngle)) + (dMiddle*Math.sin((odometryAngle)-Math.PI/2));
                odometryAngle = odometryAngle + dAngle;
            }
            lastEncoderL = currentEncoderL;
            lastEncoderR = currentEncoderR;
        }
    }
}
