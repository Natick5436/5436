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
        String[] settingNames = {"Move foundation(true = yes, false = no): ", "Park skybridge side(true) or wall side (false): "};
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
            robot.forward(1.0,-0.05,true);
            sleep(1000);
            if (t1.isSkystone()){
                finalPos = 3;
            }else{
                finalPos = 1;
            }
        }
        robot.strafe(1.0,SKYSTONE.DISTANCE_TO_STONES/2-SKYSTONE.ROBOT_WIDTH/2,true);
        robot.turn(0.6, 0,false);
        robot.approachStonesSensorR(0.15,0.02,0.2);
        if (finalPos == 1) {
            robot.strafe(0.6, 0.5, true);
        }else if(finalPos == 3){
            robot.strafe(0.6, 0.04, true);
        }/*else if(finalPos == 2){
            robot.strafe(0.6, 0.02, true);
        }*/
        robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        sleep(500);
        double blockX = robot.odo.getX();
        robot.skyClamp1.setPosition(robot.SKYCLAMP_CLOSE);
        sleep(200);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        robot.skyArm2.setPosition((robot.SKYARM2_DOWN+robot.SKYARM2_UP)/2);
        sleep(1000);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        /*robot.forward(0.5,0.05,true);
        robot.goToAbsolutePosition(1,0.6, robot.odo.getX(),SKYSTONE.FIELD_WIDTH-SKYSTONE.WALL_TO_FOUNDATION-SKYSTONE.FOUNDATION_LENGTH/2-0.07);
        robot.turn(0.6,0,false);
        robot.forward(0.5, -0.18, false);*/
        robot.angleStrafeToAbsolutePosition(1.4, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.1, SKYSTONE.FIELD_WIDTH/2+0.15, false);
        robot.angleStrafeToAbsolutePosition(1.4, SKYSTONE.FIELD_WIDTH-(SKYSTONE.START_TO_FOUNDATION-SKYSTONE.ROBOT_WIDTH/2+0.13), SKYSTONE.FIELD_WIDTH-SKYSTONE.WALL_TO_FOUNDATION-SKYSTONE.FOUNDATION_LENGTH/2+0.3, true);
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        sleep(500);
        robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
        sleep(200);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        robot.angleStrafeToAbsolutePosition(1.4, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.1, SKYSTONE.FIELD_WIDTH/2+0.15, false);
        double secondBlockY = SKYSTONE.ROBOT_WIDTH/2+0.02;
        for(int i = 1; i<finalPos; i++){
            secondBlockY += SKYSTONE.STONE_LENGTH;
        }
        robot.strafe(1, SKYSTONE.FIELD_WIDTH/2+0.15-secondBlockY, true);
        robot.approachStonesSensor(robot.sensorDistanceR,0.15,0.02,0.2);
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        sleep(500);
        robot.skyClamp1.setPosition(robot.SKYCLAMP_CLOSE);
        sleep(200);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        robot.skyArm2.setPosition((robot.SKYARM2_DOWN+robot.SKYARM2_UP)/2);
        sleep(700);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        robot.angleStrafeToAbsolutePosition(1.4, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.1, SKYSTONE.FIELD_WIDTH/2+0.15, false);
        robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        robot.angleStrafeToAbsolutePosition(1.4, SKYSTONE.FIELD_WIDTH-(SKYSTONE.START_TO_FOUNDATION-SKYSTONE.ROBOT_WIDTH/2), SKYSTONE.FIELD_WIDTH-SKYSTONE.WALL_TO_FOUNDATION-SKYSTONE.FOUNDATION_LENGTH/2+0.3, true);
        if(settings[0]){
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.strafe(1, -0.15, true);
            robot.arch(1, -0.21305, 0.087329, false);
            robot.forward(1, 0.5, true);
            robot.arch(1, -0.21305, 0.401987, false);
            robot.forward(1, -0.08, false);
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        }
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        sleep(500);
        robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
        sleep(200);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        robot.angleStrafeToAbsolutePosition(1.4, SKYSTONE.FIELD_WIDTH-3*SKYSTONE.SKYBRIDGE_LENGTH/4+0.1, SKYSTONE.FIELD_WIDTH/2+0.15, true);
    }
}
