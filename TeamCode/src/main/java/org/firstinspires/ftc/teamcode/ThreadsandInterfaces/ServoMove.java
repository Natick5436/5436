package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoMove extends Thread implements Execution {
    public static enum MovementType {INSTANT, OVER_TIME};
    private double targetPosition;
    private double time;
    private MovementType movementType;
    Servo servo;
    LinearOpMode ln;
    public ServoMove(Servo s, double tp, MovementType mt, LinearOpMode linearOpMode){
        servo = s;
        targetPosition = tp;
        time = 0;
        movementType = mt;
        ln = linearOpMode;
    }
    public ServoMove(Servo s, double tp, double tTime, MovementType mt, LinearOpMode linearOpMode){
        servo = s;
        targetPosition = tp;
        time =tTime;
        movementType = mt;
        ln = linearOpMode;
    }
    public void run(){
        double startTime = System.currentTimeMillis();
        double startPos = servo.getPosition();
        if(movementType == MovementType.INSTANT){
            servo.setPosition(targetPosition);
        }else if(movementType == MovementType.OVER_TIME){
            while(System.currentTimeMillis()-startTime < time && ln.opModeIsActive()){
                servo.setPosition(startPos + (targetPosition-startPos)*(System.currentTimeMillis()-startTime)/time);
            }
        }
    }
}
