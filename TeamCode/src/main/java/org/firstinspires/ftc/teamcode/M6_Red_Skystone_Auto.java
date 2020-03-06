package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Hardware.Vuforia;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.SKYSTONE;

@Autonomous(name = "M6 Red Skystone", group = "Autonomous")
public class M6_Red_Skystone_Auto  extends LinearOpMode {
    Mark_6 robot = new Mark_6(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        robot.initialize(hardwareMap, SKYSTONE.FIELD_WIDTH-0.23, 1.15, -Math.PI/2, false);

        t1.redMode = true;
        int itemSelected = 0;
        String[] settingNames = {"1 Skystone foundation(true = yes, false = 2 skystones", "Park skybridge side(false) or wall side (true): ", "3rd Skystone?"};
        boolean[] settings = new boolean[settingNames.length];
        boolean active = false;
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.dpad_up && !active){
                if(itemSelected <= 0){
                    itemSelected = settingNames.length-1;
                }else{
                    itemSelected--;
                }
                active = true;
            }else if(gamepad1.dpad_down && !active){
                if(itemSelected >= settingNames.length-1){
                    itemSelected = 0;
                }else{
                    itemSelected++;
                }
                active = true;
            }else if((gamepad1.dpad_left|| gamepad1.dpad_right)&& !active){
                settings[itemSelected] = !settings[itemSelected];
                active = true;
            }else if(active&&!gamepad1.dpad_right&&!gamepad1.dpad_left&&!gamepad1.dpad_up&&!gamepad1.dpad_down){
                active = false;
            }
            if(!gamepad1.a) {
                for (int i = 0; i < settingNames.length; i++) {
                    if (i == itemSelected) {
                        telemetry.addData(">>>" + settingNames[i], settings[i]);
                    } else {
                        telemetry.addData(settingNames[i], settings[i]);
                    }
                }
                telemetry.update();
            }
        }
        robot.odo.start();
        runtime.reset();
        waitForStart();
        robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
        robot.skyClamp2.setPosition(robot.SKYCLAMP_OPEN);
       // t1.exit = true;
       // Thread.sleep(600);
       // t1.exit = false;
       // robot.forward(0.5, 0.01, true);
       // sleep(1000);
        int finalPos = 2;
        if (t1.pos != 2){
            robot.forward(0.25,-0.10,true);
            sleep(1000);
            if (t1.isSkystone()){
                finalPos = 3;
            }else{
                finalPos = 1;
            }
        }
        robot.strafe(1.0,SKYSTONE.DISTANCE_TO_STONES/2-SKYSTONE.ROBOT_WIDTH/2,true);
        robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        robot.turn(0.5, 0,false);
        if (finalPos == 1) {
            robot.approachStonesSensor(robot.sensorDistanceR, 0.2,0.03,0.25, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.strafe(0.6, 0.35, true);
            robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
            //robot.forward(-0.1);
            sleep(700);
            //robot.stopDrive();
        }else if(finalPos == 3){
            robot.approachStonesSensor(robot.sensorDistanceR, 0.24,0.05,0.25, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
            robot.forward(0.2, -0.04, false);
            //robot.forward(-0.1);
            sleep(700);
            //robot.stopDrive();
        }else if(finalPos == 2){
            robot.approachStonesSensor(robot.sensorDistanceR, 0.24,0.05,0.25, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
            robot.forward(0.2, -0.04, false);
            //robot.forward(-0.1);
            sleep(700);
            //robot.stopDrive();
            //robot.strafe(0.6, 0.02, true);
        }
        double blockX = robot.odo.getX();
        robot.skyClamp1.setPosition(robot.SKYCLAMP_CLOSE);
        sleep(500);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        robot.skyArm2.setPosition((robot.SKYARM2_DOWN));
        sleep(1200);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        /*robot.forward(0.5,0.05,true);
        robot.goToAbsolutePosition(1,0.6, robot.odo.getX(),SKYSTONE.FIELD_WIDTH-SKYSTONE.WALL_TO_FOUNDATION-SKYSTONE.FOUNDATION_LENGTH/2-0.07);
        robot.turn(0.6,0,false);
        robot.forward(0.5, -0.18, false);*/
        robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.15, SKYSTONE.FIELD_WIDTH/2+0.15, false);
        robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        if(settings[0]) {
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-(SKYSTONE.START_TO_FOUNDATION-SKYSTONE.ROBOT_WIDTH/2)+0.05, SKYSTONE.FIELD_WIDTH-SKYSTONE.WALL_TO_FOUNDATION-SKYSTONE.FOUNDATION_LENGTH/2+0.1, true);
        }else{
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-0.8, SKYSTONE.FIELD_WIDTH/2+0.5, true);
        }
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
        sleep(100);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        if(settings[0]){
            robot.forward(0.45, -0.3, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            sleep(600);
            robot.arch(1, -0.21305, 0.087329, false);
            robot.forward(1, 0.46, true);
            robot.arch(1, -0.21305, 0.401987, false);
            robot.forward(1, -0.08, false);
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            sleep(300);
        }else {
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.25, SKYSTONE.FIELD_WIDTH/2+0.15, false);
            double secondBlockY = SKYSTONE.ROBOT_WIDTH / 2 + 0.03;
            for (int i = 1; i < finalPos; i++) {
                secondBlockY += SKYSTONE.STONE_LENGTH;
            }
            if (Math.abs(robot.getHeading() - 0) > 0.2) {
                robot.turn(0.3, 0, false);
            }
            if(finalPos != 1) {
                robot.strafe(1, SKYSTONE.FIELD_WIDTH / 2 + 0.15 - secondBlockY, true);
            }else{
                robot.strafe(1, SKYSTONE.FIELD_WIDTH / 2 + 0.15 - secondBlockY-0.2, true);
                robot.strafe(0.5);
                sleep(1000);
                robot.stopDrive();
            }
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            robot.approachStonesSensor(robot.sensorDistanceR, 0.24,0.05,0.25, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
            robot.forward(0.2, -0.04, false);
            //robot.forward(-0.1);
            sleep(700);
            //robot.stopDrive();
            robot.skyClamp1.setPosition(robot.SKYCLAMP_CLOSE);
            sleep(500);
            robot.skyArm1.setPosition(robot.SKYARM1_UP);
            robot.skyArm2.setPosition((robot.SKYARM2_DOWN + robot.SKYARM2_UP) / 2);
            sleep(800);
            robot.skyArm2.setPosition(robot.SKYARM2_UP);
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH - 3 * SKYSTONE.SKYBRIDGE_LENGTH / 4 + 0.15, SKYSTONE.FIELD_WIDTH / 2 + 0.15, false);
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-0.8, SKYSTONE.FIELD_WIDTH/2+0.5, true);
            robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
            robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
            sleep(100);
            robot.skyArm1.setPosition(robot.SKYARM1_UP);
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            if(settings[2]){
                robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.25, SKYSTONE.FIELD_WIDTH/2+0.15, false);
                double thirdBlockY = SKYSTONE.ROBOT_WIDTH / 2 + 0.03;
                if(finalPos != 3) {
                    for (int i = 1; i < 6; i++) {
                        thirdBlockY += SKYSTONE.STONE_LENGTH;
                    }
                }else{
                    for (int i = 1; i < 5; i++) {
                        thirdBlockY += SKYSTONE.STONE_LENGTH;
                    }
                }
                if (Math.abs(robot.getHeading() - 0) > 0.2) {
                    robot.turn(0.3, 0, false);
                }
                robot.strafe(1, SKYSTONE.FIELD_WIDTH / 2 + 0.15 - thirdBlockY, true);
                robot.foundation.setPosition(robot.FOUNDATION_OPEN);
                robot.approachStonesSensor(robot.sensorDistanceR, 0.24,0.05,0.25, false);
                robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
                robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
                robot.forward(0.2, -0.04, false);
                //robot.forward(-0.1);
                sleep(700);
                //robot.stopDrive();
                robot.skyClamp1.setPosition(robot.SKYCLAMP_CLOSE);
                sleep(500);
                robot.skyArm1.setPosition(robot.SKYARM1_UP);
                robot.skyArm2.setPosition((robot.SKYARM2_DOWN + robot.SKYARM2_UP) / 2);
                sleep(800);
                robot.skyArm2.setPosition(robot.SKYARM2_UP);
                robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH - 3 * SKYSTONE.SKYBRIDGE_LENGTH / 4 + 0.15, SKYSTONE.FIELD_WIDTH / 2 + 0.15, false);
                robot.foundation.setPosition(robot.FOUNDATION_OPEN);
                robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-0.8, SKYSTONE.FIELD_WIDTH/2+0.5, true);
                robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
                robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
                sleep(100);
                robot.skyArm1.setPosition(robot.SKYARM1_UP);
                robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            }
        }
        if(settings[1]){
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH-SKYSTONE.SKYBRIDGE_LENGTH/4+0.05, SKYSTONE.FIELD_WIDTH/2+0.1, true);
        }else {
            robot.angleStrafeToAbsolutePosition(2, SKYSTONE.FIELD_WIDTH - 3 * SKYSTONE.SKYBRIDGE_LENGTH / 4, SKYSTONE.FIELD_WIDTH / 2 + 0.05, true);
        }
    }
}
