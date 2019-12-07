package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "M5 Blue Build Site", group = "Autonomous")
public class M5_Blue_Build_Auto extends LinearOpMode {

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
    final double QUIT_X = (SKYBRIDGE_LENGTH/4);
    final double QUIT_Y = FIELD_WIDTH/2;
    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap, ROBOT_WIDTH/2, FIELD_WIDTH - (WALL_TO_FOUNDATION + FOUNDATION_LENGTH/2), 0, false);

        runtime.reset();
        waitForStart();

        robot.forward(0.25, START_TO_FOUNDATION-ROBOT_WIDTH);
        robot.setGrab(1);
        sleep(1000);
        if(runtime.seconds()> QUIT_TIME){
            robot.setGrab(0);
            robot.goToAbsolutePosition(0.25, QUIT_X, QUIT_Y);
            return;
        }
        robot.forward(0.25, -((FOUNDATION_LENGTH/2)-0.05));
        if(runtime.seconds()> QUIT_TIME){
            robot.setGrab(0);
            robot.goToAbsolutePosition(0.25, QUIT_X, QUIT_Y);
            return;
        }
        robot.turn(0.25, Math.PI/2);
        if(runtime.seconds()> QUIT_TIME){
            robot.setGrab(0);
            robot.goToAbsolutePosition(0.25, QUIT_X, QUIT_Y);
            return;
        }
        robot.forward(0.25, FOUNDATION_WIDTH);
        robot.setGrab(0);
        robot.goToAbsolutePosition(0.25, QUIT_X, QUIT_Y);
    }
}
