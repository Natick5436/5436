package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class Movement<DEVICE extends HardwareDevice> extends Thread{

    private LinearOpMode ln;
    private DEVICE device;
    private double startTime;
    private double commandStartTime;
    private double commandTimeDuration;
    private Double targetValue;
    public Movement(DEVICE d, LinearOpMode linearOpMode){
        device = d;
        ln = linearOpMode;
        startTime = System.currentTimeMillis();
        commandStartTime = System.currentTimeMillis();
        commandTimeDuration = 0;
        targetValue = Double.NaN;
    }

    public void setTargetValue(double tV, double tTime){
        targetValue = tV;
        commandTimeDuration = tTime;
        commandStartTime = System.currentTimeMillis();
    }

    public void run(){
        startTime = System.currentTimeMillis();
        while(ln.opModeIsActive()) {
            if (device instanceof Servo) {

            } else if (device instanceof DcMotor) {

            }
        }
    }
}
