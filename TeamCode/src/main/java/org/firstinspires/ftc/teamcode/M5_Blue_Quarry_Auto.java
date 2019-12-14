package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "M5 Blue Quarry", group = "Autonomous")
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

    final double CLAMP_CLOSE = 0.9;
    final double CLAMP_OPEN = 0.45;

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
        robot.initialize(hardwareMap, ROBOT_WIDTH/2, QUARRY_LENGTH/2, 0, false);

        runtime.reset();
        waitForStart();

        robot.updateVuforia();
        double skystoneDelta;
        if (robot.isSkystone())
            skystoneDelta = robot.getSkystonePosition().get(X)*metersPerMm;
        else
            skystoneDelta = 0;

        while(robot.isSkystone() && Math.abs(MIDDLE_OF_SCREEN-robot.getSkystonePosition().get(Y)*metersPerMm) > SKYSTONE_ACCURACY){
            robot.strafe(Math.abs(MIDDLE_OF_SCREEN-robot.getSkystonePosition().get(Y)*metersPerMm)/(MIDDLE_OF_SCREEN-robot.getSkystonePosition().get(Y)*metersPerMm));
            robot.updateVuforia();
            telemetry.update();
            if(isStopRequested())return;
        }
        robot.setOdometryPosition(ROBOT_WIDTH/2, QUARRY_LENGTH/2 - skystoneDelta);

        robot.setArm(1, ARM_MID);
        robot.clamp.setPosition(CLAMP_OPEN);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);

        robot.forward(1, DISTANCE_TO_STONES-ARM_LENGTH-ROBOT_WIDTH);
        robot.clamp.setPosition(CLAMP_CLOSE);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.setArm(1, ARM_IN);

        robot.goToAbsolutePosition(1, (3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH-(WALL_TO_FOUNDATION+FOUNDATION_LENGTH/2));
        robot.turn(1, 0);

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

        robot.goToAbsolutePosition(1, ROBOT_WIDTH/2, QUARRY_LENGTH/2);
        robot.turn(1, 0);

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
                robot.setOdometryPosition(ROBOT_WIDTH/2, QUARRY_LENGTH/2 - (skystoneDelta-robot.getSkystonePosition().get(X)*metersPerMm));
                robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
                return;
            }
        }
        robot.setOdometryPosition(ROBOT_WIDTH/2, QUARRY_LENGTH/2 - skystoneDelta);

        robot.setArm(1, ARM_MID);
        robot.clamp.setPosition(CLAMP_OPEN);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
            return;
        }

        robot.forward(1, DISTANCE_TO_STONES-ARM_LENGTH);
        robot.clamp.setPosition(CLAMP_CLOSE);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.setArm(1, ARM_IN);

        if(runtime.seconds()> QUIT_TIME){
            robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
            return;
        }

        robot.goToAbsolutePosition(1, (3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH-(WALL_TO_FOUNDATION+FOUNDATION_LENGTH/2));
        robot.turn(1, 0);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);
        robot.clamp.setPosition(CLAMP_OPEN);

        robot.setArm(1, ARM_MID);
        robot.flip.setPosition(FLIP_STORE);
        robot.clamp.setPosition(CLAMP_CLOSE);
        robot.setArm(1, ARM_IN);

        robot.goToAbsolutePosition(1, QUIT_X, QUIT_Y);
        robot.targetsSkyStone.deactivate();
    }
}
