package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "M5 Red Build Site", group = "Autonomous")
public class M5_Red_Build_Auto extends LinearOpMode {

    Mark_5 robot = new Mark_5(this);

    final double FIELD_WIDTH = 3.5814;
    final double ROBOT_WIDTH = 0.4572;
    final double DISTANCE_TO_STONES = 1.1938;
    final double QUARRY_LENGTH = 1.2319;
    final double ARM_LENGTH = 0.16;
    final double FOUNDATION_LENGTH = 0.8763;
    final double FOUNDATION_WIDTH = 0.4699;
    final double WALL_TO_FOUNDATION = 0.1016;
    final double START_TO_FOUNDATION = 1.20015;

    final double metersPerMm = 0.001;

    final int X = 0;
    final int Y = 1;
    final int Z = 2;

    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap, FIELD_WIDTH-ROBOT_WIDTH, FIELD_WIDTH - (WALL_TO_FOUNDATION + FOUNDATION_LENGTH/2), Math.PI);

        waitForStart();

        robot.forward(1, START_TO_FOUNDATION-ROBOT_WIDTH);
        robot.setGrab(1);
        robot.forward(1, -((FOUNDATION_LENGTH/2)-0.05));
        robot.turn(1, Math.PI/2);
        robot.forward(1, FOUNDATION_WIDTH);
        robot.setGrab(0);
        robot.goToAbsolutePosition(1, FIELD_WIDTH-(ROBOT_WIDTH/2+0.03), FIELD_WIDTH/2);
    }
}
