package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Disabled
@Autonomous (name = "Mark 4 Auto", group = "Autonomous")
public class Mark_Four_Autonomous extends LinearOpMode {
    Mark_4 robot = new Mark_4(this,hardwareMap);
    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap, 0, 0, 0);
        waitForStart();
        //62.3 cm from wall
        //robot.goToDeltaPosition(0.5,Math.PI/2, 1);
        //robot.goToAbsolutePosition(0,0, 0.5);
        telemetry.addData("Red  ", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.METER)));
        telemetry.update();
        boolean distanceReached = false;
        double goalDistance = 0.3;
        double goalDistanceAcc = 0.01;
        robot.middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(!distanceReached){
            Double deltaDistance = new Double(robot.sensorDistance.getDistance(DistanceUnit.METER));
            distanceReached = false;
            //robot strafe no more
            if(deltaDistance.isNaN()){
                robot.middle.setPower(0.5);
            }
            if (robot.sensorDistance.getDistance(DistanceUnit.METER)-goalDistance > 0){
                robot.middle.setPower(0.2);
            }
            if (robot.sensorDistance.getDistance(DistanceUnit.METER)-goalDistance < 0){
                robot.middle.setPower(-0.2);
            }
            if(Math.abs(robot.sensorDistance.getDistance(DistanceUnit.METER) - goalDistance) < goalDistanceAcc){
                distanceReached = true;
            }
            if (distanceReached){
                robot.middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (isStopRequested()){
                return;
            }
        }
        robot.middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.middle.setPower(0);
        telemetry.addData("Sky Stone", robot.skystone);
        robot.relativeLayout.post(new Runnable() {
            public void run() {
                robot.relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, robot.values));
            }
        });

        telemetry.update();
        robot.relativeLayout.post(new Runnable() {
            public void run() {
                robot.relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}


