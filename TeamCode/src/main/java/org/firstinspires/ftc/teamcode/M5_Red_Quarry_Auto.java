package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "M5 Red Quarry", group = "Autonomous")
public class M5_Red_Quarry_Auto extends LinearOpMode {

    Mark_5 robot = new Mark_5(this);

    final double FIELD_WIDTH = 3.5814;
    final double ROBOT_WIDTH = 0.4572;
    final double DISTANCE_TO_STONES = 1.1938;
    final double QUARRY_LENGTH = 1.2319;

    final double metersPerInch = 0.0254;

    final int X = 0;
    final int Y = 1;
    final int Z = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, ROBOT_WIDTH, QUARRY_LENGTH / 2, 0);

        waitForStart();

        robot.updateVuforia();
        double skystoneDelta;
        if (robot.isSkystone())
            skystoneDelta = robot.getSkystonePosition().get(X)/metersPerInch;
        else
            skystoneDelta = 0;

        while(robot.isSkystone() && Math.abs(robot.getSkystonePosition().get(X)/metersPerInch) > robot.distanceAccuracy){
            robot.strafe(Math.abs(robot.getSkystonePosition().get(X)/metersPerInch)/(robot.getSkystonePosition().get(X)/metersPerInch));
            robot.updateVuforia();
            telemetry.update();
        }
        robot.setOdometryPosition(ROBOT_WIDTH, QUARRY_LENGTH/2 - skystoneDelta);
        robot.forward(0.5, DISTANCE_TO_STONES);
    }
}
