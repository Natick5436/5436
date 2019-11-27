package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "M5 Blue Build Site", group = "Autonomous")
public class M5_Blue_Build_Auto extends LinearOpMode {

    Mark_5 robot = new Mark_5(this);

    final double FIELD_WIDTH = 3.5814;
    final double ROBOT_WIDTH = 0.4572;
    final double DISTANCE_TO_STONES = 1.1938;
    final double QUARRY_LENGTH = 1.2319;
    final double ARM_LENGTH = 0.25;
    final double FOUNDATION_LENGTH = 0.8763;
    final double WALL_TO_FOUNDATION = 0.1016;

    final double metersPerInch = 0.0254;

    @Override
    public void runOpMode(){
        robot.initialize(hardwareMap, ROBOT_WIDTH, FIELD_WIDTH - (WALL_TO_FOUNDATION + FOUNDATION_LENGTH/2), 0);

        waitForStart();
    }
}
