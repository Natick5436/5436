package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Hardware.Vuforia;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.SKYSTONE;

@Autonomous(name = "M6 Blue Skystone", group = "Autonomous")
public class M6_Blue_Skystone_Auto  extends LinearOpMode {
    Mark_6 robot = new Mark_6(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        robot.initialize(hardwareMap, 0.22, 0.78, Math.PI / 2, false);

        t1.redMode = true;
        int itemSelected = 0;
        String[] settingNames = {"Move foundation(true = yes, false = no): ", "Park skybridge side(true) or wall side (false): "};
        boolean[] settings = new boolean[settingNames.length];
        boolean active = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up && !active) {
                if (itemSelected <= 0) {
                    itemSelected = settingNames.length - 1;
                } else {
                    itemSelected--;
                }
                active = true;
            } else if (gamepad1.dpad_down && !active) {
                if (itemSelected >= settingNames.length - 1) {
                    itemSelected = 0;
                } else {
                    itemSelected++;
                }
                active = true;
            } else if ((gamepad1.dpad_left || gamepad1.dpad_right) && !active) {
                settings[itemSelected] = !settings[itemSelected];
                active = true;
            } else if (active && !gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad1.dpad_down) {
                active = false;
            }
            if (!gamepad1.a) {
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
        if (t1.pos != 2) {
            robot.forward(1.0, -0.05, true);
            sleep(1000);
            if (t1.isSkystone()) {
                finalPos = 3;
            } else {
                finalPos = 1;
            }
        }
        robot.strafe(1.0, SKYSTONE.DISTANCE_TO_STONES / 2 - SKYSTONE.ROBOT_WIDTH / 2, true);
        robot.turn(0.6, 0, false);
        robot.approachStonesSensor(robot.sensorDistanceB,0.15,0.02,0.3);
        robot.turn(0.4, 0, false);
        if (finalPos == 1) {
            robot.strafe(0.6, 0.5, true);
        } else if (finalPos == 3) {
            robot.strafe(0.6, -0.2, true);
        }else if(finalPos == 2) {
            robot.strafe(0.6, -0.2, true);
        }
        robot.skyArm2.setPosition(robot.SKYARM2_DOWN);
        sleep(500);
        double blockX = robot.odo.getX();
        robot.skyClamp2.setPosition(robot.SKYCLAMP_CLOSE);
        sleep(200);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        sleep(700);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        /*robot.forward(0.5,0.05,true);
        robot.goToAbsolutePosition(1,0.6, robot.odo.getX(),SKYSTONE.FIELD_WIDTH-SKYSTONE.WALL_TO_FOUNDATION-SKYSTONE.FOUNDATION_LENGTH/2-0.07);
        robot.turn(0.6,0,false);
        robot.forward(0.5, -0.18, false);*/
        robot.angleStrafeToAbsolutePosition(1.4, 3* SKYSTONE.SKYBRIDGE_LENGTH / 4 - 0.25, SKYSTONE.FIELD_WIDTH / 2 + 0.15, false);
        robot.angleStrafeToAbsolutePosition(1.4, (SKYSTONE.START_TO_FOUNDATION - SKYSTONE.ROBOT_WIDTH / 2), SKYSTONE.FIELD_WIDTH - SKYSTONE.WALL_TO_FOUNDATION - SKYSTONE.FOUNDATION_LENGTH / 2 + 0.2, true);
        robot.skyArm2.setPosition(robot.SKYARM2_DOWN);
        sleep(500);
        robot.skyClamp2.setPosition(robot.SKYCLAMP_OPEN);
        sleep(200);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        robot.angleStrafeToAbsolutePosition(1.4, 3 * SKYSTONE.SKYBRIDGE_LENGTH / 4 - 0.2, SKYSTONE.FIELD_WIDTH / 2 + 0.15, false);
        double secondBlockY = SKYSTONE.ROBOT_WIDTH / 2 + 0.02;
        for (int i = 1; i < finalPos; i++) {
            secondBlockY += SKYSTONE.STONE_LENGTH;
        }
        robot.strafe(1, SKYSTONE.FIELD_WIDTH / 2 + 0.15 - secondBlockY, true);
        robot.approachStonesSensor(robot.sensorDistanceB,0.15,0.02,0.3);
        robot.strafe(1, 0.19, true);
        robot.skyArm2.setPosition(robot.SKYARM2_DOWN);
        sleep(500);
        robot.skyClamp2.setPosition(robot.SKYCLAMP_CLOSE);
        sleep(200);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        robot.skyArm1.setPosition(robot.SKYARM1_DOWN);
        sleep(700);
        robot.skyArm1.setPosition(robot.SKYARM1_UP);
        robot.angleStrafeToAbsolutePosition(1.4, 3 * SKYSTONE.SKYBRIDGE_LENGTH / 4 - 0.2, SKYSTONE.FIELD_WIDTH / 2 + 0.15, false);
        robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        robot.angleStrafeToAbsolutePosition(1.4, (SKYSTONE.START_TO_FOUNDATION - SKYSTONE.ROBOT_WIDTH / 2), SKYSTONE.FIELD_WIDTH - SKYSTONE.WALL_TO_FOUNDATION - SKYSTONE.FOUNDATION_LENGTH / 2 + 0.2, true);
        if (settings[0]) {
            robot.strafe(1, 0.1, true);
            robot.forward(1, -(SKYSTONE.START_TO_FOUNDATION - SKYSTONE.ROBOT_WIDTH) + 0.2, false);
            robot.forward(0.4, -0.18, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.strafe(1, 0.1, true);
            robot.arch(1, 0.21305, 0.087329, false);
            robot.forward(0.7, 0.5, true);
            telemetry.addData("Forward is done", "Yes");
            telemetry.update();
            robot.arch(1, 0.21305, 0.401987, false);
            robot.forward(1, -0.08, false);
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        }
        robot.skyArm2.setPosition(robot.SKYARM2_DOWN);
        sleep(500);
        robot.skyClamp2.setPosition(robot.SKYCLAMP_OPEN);
        sleep(200);
        robot.skyArm2.setPosition(robot.SKYARM2_UP);
        robot.angleStrafeToAbsolutePosition(1, 3 * SKYSTONE.SKYBRIDGE_LENGTH / 4 - 0.2, SKYSTONE.FIELD_WIDTH / 2 + 0.15, true);
    }
}
