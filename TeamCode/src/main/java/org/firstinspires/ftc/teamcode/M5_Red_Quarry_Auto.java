package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.*;

@Autonomous(name = "M5 Red Quarry", group = "Autonomous")
public class M5_Red_Quarry_Auto extends LinearOpMode {

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
    final double QUIT_X = FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4);
    final double QUIT_Y = FIELD_WIDTH/2;

    final double SKYSTONE_ACCURACY = 0.5;
    final double MIDDLE_OF_SCREEN = -0.280;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH / 2, Math.PI, false);

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
            robot.turn(0.2, 11*Math.PI/12, true);
            sleep(2000);
            if(t1.isSkystone()){
                stonePosition = 3;
            }else{
                robot.turn(0.2, 7*Math.PI/6, true);
                sleep(2000);
                if(t1.isSkystone()){
                    stonePosition = 1;
                }else{
                    stonePosition = 1;
                }
            }
        }
        telemetry.addData("StonePos", stonePosition);
        telemetry.update();
        if(stonePosition == 2){
            robot.turn(0.2, Math.PI, true);
            robot.clamp.setPosition(CLAMP_OPEN);
            robot.setArm(0.4, -2350);
            robot.forward(0.2, 0.35);
            robot.turn(0.1, 23*Math.PI/24);
            robot.clamp.setPosition(CLAMP_CLOSE);
            sleep(1000);
            robot.setArm(0.3,-2000);
            robot.forward(0.3, -0.4);
            robot.stopDrive();
        }
        if(stonePosition == 3){
            robot.turn(0.2,Math.PI,true);
            robot.clamp.setPosition(CLAMP_OPEN);
            robot.setArm(0.4,-2350);
            robot.turn(0.4, 11*Math.PI/12, true);
            robot.forward(0.2,0.35);
            //robot.turn();
            robot.clamp.setPosition(CLAMP_CLOSE);
            sleep(1000);
            robot.setArm(0.3,-2000);
            robot.forward(0.3,-0.4);
            robot.stopDrive();
        }
        if (stonePosition != 2 && stonePosition != 3){
            //robot.turn(0.3, 25*Math.PI/24);
            robot.clamp.setPosition(CLAMP_OPEN);
            robot.setArm(0.4,-2400);
            robot.forward(0.2,0.38);
            //robot.turn(0.3, 49*Math.PI/48);
            robot.clamp.setPosition(CLAMP_CLOSE);
            sleep(1000);
            robot.setArm(0.3,-2000);
            robot.forward(0.3,-0.4);
            robot.stopDrive();
        }


        /*double skystoneDelta;
        double startTime = runtime.seconds();
        while(!t1.isSkystone()){
            double deltaTime = runtime.seconds()-startTime;
            if(((int)deltaTime/2)%7 == 0) {
                robot.strafe(-0.4);
            }else if(((int)deltaTime/2)%7 == 1){
                robot.stopDrive();
            }else if(((int)deltaTime/2)%7 == 2){
                robot.strafe(0.4);
            }else if(((int)deltaTime/2)%7 == 3){
                robot.stopDrive();
            }else if(((int)deltaTime/2)%7 == 4){
                robot.strafe(-0.4);
            }else if(((int)deltaTime/2)%7 == 5){
                robot.stopDrive();
            }else{
                robot.strafe(0.4);
            }
            if(isStopRequested() ){
                return;
            }
            telemetry.addData("Searching:","first loop");
            telemetry.addData("robot.isSkystone()", t1.isSkystone());
            telemetry.update();
        }
        if (t1.isSkystone())
            skystoneDelta = MIDDLE_OF_SCREEN-t1.getSkystonePosition().get(Y)*metersPerMm;
        else
            skystoneDelta = 0;
        telemetry.addData("Skystone delta position", skystoneDelta);
        telemetry.addData("skystone pos", t1.getSkystonePosition().get(Y)*metersPerMm);
        telemetry.update();
        sleep(5000);
        while(t1.isSkystone() && Math.abs(MIDDLE_OF_SCREEN-t1.getSkystonePosition().get(Y)*metersPerMm) > SKYSTONE_ACCURACY){
            robot.strafe(-Math.abs(MIDDLE_OF_SCREEN-t1.getSkystonePosition().get(Y)*metersPerMm)/(MIDDLE_OF_SCREEN-t1.getSkystonePosition().get(Y)*metersPerMm));
            telemetry.update();
            if(isStopRequested())return;
        }
        robot.setOdometryPosition(FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH/2 + skystoneDelta);*/

        /*robot.setArm(1, ARM_MID);
        robot.clamp.setPosition(CLAMP_OPEN);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);

        robot.clamp.setPosition(CLAMP_CLOSE);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.setArm(1, ARM_IN);

        robot.goToAbsolutePosition(1, FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH-(WALL_TO_FOUNDATION+FOUNDATION_LENGTH/2));
        robot.turn(1, Math.PI);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);
        robot.clamp.setPosition(CLAMP_OPEN);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.clamp.setPosition(CLAMP_CLOSE);
        robot.setArm(1, ARM_IN);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
            return;
        }

        robot.goToAbsolutePosition(1, FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH/2);
        robot.turn(1, Math.PI);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
            return;
        }

        robot.updateVuforia();
        if (robot.isSkystone())
            skystoneDelta = robot.getSkystonePosition().get(X)*metersPerMm;
        else
            skystoneDelta = 0;

        while(robot.isSkystone() && Math.abs(MIDDLE_OF_SCREEN-robot.getSkystonePosition().get(Y)*metersPerMm) > SKYSTONE_ACCURACY){
            robot.strafe(Math.abs(MIDDLE_OF_SCREEN-robot.getSkystonePosition().get(Y)*metersPerMm)/(MIDDLE_OF_SCREEN-robot.getSkystonePosition().get(Y)*metersPerMm));
            robot.updateVuforia();
            telemetry.update();
            if(isStopRequested())return;
            if(runtime.seconds()> QUIT_TIME){
                robot.setOdometryPosition(ROBOT_WIDTH/2, QUARRY_LENGTH/2 + (skystoneDelta-robot.getSkystonePosition().get(X)*metersPerMm));
                robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
                return;
            }
        }
        robot.setOdometryPosition(FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH/2 + skystoneDelta);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
            return;
        }

        robot.setArm(1, ARM_MID);
        robot.clamp.setPosition(CLAMP_OPEN);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);

        robot.forward(1, DISTANCE_TO_STONES-ARM_LENGTH);
        robot.clamp.setPosition(CLAMP_CLOSE);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.setArm(1, ARM_IN);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH/2);
            return;
        }

        robot.goToAbsolutePosition(1, FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH-(WALL_TO_FOUNDATION+FOUNDATION_LENGTH/2));
        robot.turn(1, Math.PI);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH/2);
            return;
        }

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);
        robot.clamp.setPosition(CLAMP_OPEN);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.clamp.setPosition(CLAMP_CLOSE);
        robot.setArm(1, ARM_IN);

        robot.goToAbsolutePosition(1, FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH/2);*/
    }
}
