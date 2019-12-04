package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    final int ARM_OUT = -2520;
    final int ARM_MID = -1260;
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
    final double QUIT_X = FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4);
    final double QUIT_Y = FIELD_WIDTH/2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH / 2, Math.PI);

        runtime.reset();
        waitForStart();

        robot.updateVuforia();
        double skystoneDelta;
        if (robot.isSkystone())
            skystoneDelta = robot.getSkystonePosition().get(X)*metersPerMm;
        else
            skystoneDelta = 0;

        while(robot.isSkystone() && Math.abs(robot.getSkystonePosition().get(X)*metersPerMm) > robot.distanceAccuracy){
            robot.strafe(Math.abs(robot.getSkystonePosition().get(X)*metersPerMm)/(robot.getSkystonePosition().get(X)*metersPerMm));
            robot.updateVuforia();
            telemetry.update();
        }
        robot.setOdometryPosition(FIELD_WIDTH-ROBOT_WIDTH/2, QUARRY_LENGTH/2 + skystoneDelta);

        robot.setArm(1, ARM_MID);
        robot.clamp.setPosition(CLAMP_OPEN);
        robot.flip.setPosition(FLIP_COLLECT);
        robot.setArm(1, ARM_OUT);

        robot.forward(1, DISTANCE_TO_STONES-ARM_LENGTH);
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

        while(robot.isSkystone() && Math.abs(robot.getSkystonePosition().get(X)*metersPerMm) > robot.distanceAccuracy){
            robot.strafe(Math.abs(robot.getSkystonePosition().get(X)*metersPerMm)/(robot.getSkystonePosition().get(X)*metersPerMm));
            robot.updateVuforia();
            telemetry.update();
            if(isStopRequested())break;
            if(runtime.seconds()> QUIT_TIME){
                robot.setOdometryPosition(ROBOT_WIDTH, QUARRY_LENGTH/2 + (skystoneDelta-robot.getSkystonePosition().get(X)*metersPerMm));
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

        robot.goToAbsolutePosition(1, FIELD_WIDTH-(3*SKYBRIDGE_LENGTH/4), FIELD_WIDTH/2);
    }
}
