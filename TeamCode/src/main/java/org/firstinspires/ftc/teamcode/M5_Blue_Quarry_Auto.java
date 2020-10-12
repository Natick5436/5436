package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "M5 Blue Quarry", group = "Autonomous")
@Disabled
public class M5_Blue_Quarry_Auto extends LinearOpMode {

    Mark_5 robot = new Mark_5(this);
    ElapsedTime runtime = new ElapsedTime();

    final double FIELD_WIDTH = 3.5814;
    final double ROBOT_WIDTH = 0.4572;
    final double DISTANCE_TO_STONES = 1.1938;
    final double QUARRY_LENGTH = 1.2319;
    final double ARM_LENGTH = 0.16;
    final double FOUNDATION_LENGTH = 0.8763;
    final double FOUNDATION_WIDTH = 0.4699;
    final double WALL_TO_FOUNDATION = 0.1016;
    final double START_TO_FOUNDATION = 1.20015;
    final double SKYBRIDGE_LENGTH = 1.168;

    final int ARM_OUT = 2111;
    final int ARM_MID = 954;
    final int ARM_IN = 0;

    final double CLAMP_CLOSE = 0.45;
    final double CLAMP_OPEN = 0.9;

    final double FLIP_COLLECT = 0.33;
    final double FLIP_STORE = 1;

    final double metersPerMm = 0.001;

    final int X = 0;
    final int Y = 1;
    final int Z = 2;

    final double QUIT_TIME = 25;
    final double QUIT_X = (3*SKYBRIDGE_LENGTH/4);
    final double QUIT_Y = FIELD_WIDTH/2;

    final double SKYSTONE_ACCURACY = 0.5;
    final double MIDDLE_OF_SCREEN = -11;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH / 2, 0, false);

        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        while(!t1.isReady()){
            if(isStopRequested()){
                return;
            }
        }
        runtime.reset();
        waitForStart();

        robot.setGrab(1);
        robot.forward(0.25, 0.3);
        robot.stopDrive();
        sleep(2000);
        int stonePosition = 2;
        if(t1.isSkystone()){
            stonePosition = 2;
        }else{
            robot.turn(0.2, -1*Math.PI/12, false);
            sleep(2000);
            if(t1.isSkystone()){
                stonePosition = 3;
            }else{
                robot.turn(0.2, 1*Math.PI/6, false);
                sleep(2000);
                if(t1.isSkystone()){
                    stonePosition = 1;
                }else{
                    stonePosition = 2;
                }
            }
        }
        telemetry.addData("StonePos", stonePosition);
        telemetry.update();
        if(stonePosition == 1){
            robot.turn(0.3, 1*Math.PI/24);
            robot.clamp.setPosition(CLAMP_OPEN);
            robot.setArm(0.4,-2200);
            robot.forward(0.2,0.38);
            //robot.turn(0.3, 49*Math.PI/48);
            robot.clamp.setPosition(CLAMP_CLOSE);
            sleep(1000);
            robot.setArm(0.3,-2000);
            robot.forward(0.3,-0.4);
            robot.stopDrive();
        }else if(stonePosition == 2){
            robot.turn(0.2, 0, false);
            robot.clamp.setPosition(CLAMP_OPEN);
            robot.setArm(0.4, -2200);
            robot.forward(0.2, 0.35);
            robot.turn(0.1, -1*Math.PI/24);
            robot.clamp.setPosition(CLAMP_CLOSE);
            sleep(1000);
            robot.setArm(0.3,-2000);
            robot.forward(0.3, -0.4);
            robot.stopDrive();
        }else if(stonePosition == 3){
            robot.clamp.setPosition(CLAMP_OPEN);
            robot.setArm(0.4,-2200);
            robot.turn(0.4, -1*Math.PI/12, false);
            robot.forward(0.2,0.35);
            robot.clamp.setPosition(CLAMP_CLOSE);
            sleep(1000);
            robot.setArm(0.3,-2000);
            robot.forward(0.3,-0.4);
            robot.stopDrive();
        }
        robot.turn(0.3, 9*Math.PI/24,false);
        robot.forward(0.3, 1.2);
        robot.stopDrive();
        robot.turn(0.3, Math.PI/2);
        robot.forward(0.3, -0.2);
        robot.strafe( 1);
        sleep(1000);
        robot.stopDrive();
        robot.targetsSkyStone.deactivate();
    }
}
