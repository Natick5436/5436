package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Test M6", group = "Autonomous")
public class M6_Autonomous extends LinearOpMode {

    Mark_6 robot = new Mark_6(this);
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
    final double QUIT_X = (3 * SKYBRIDGE_LENGTH / 4);
    final double QUIT_Y = FIELD_WIDTH / 2;

    final double SKYSTONE_ACCURACY = 0.5;
    final double MIDDLE_OF_SCREEN = -11;
    int stonePosition;


    @Override
    public void runOpMode() throws InterruptedException {

        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        robot.initialize(hardwareMap, 0, 0, 0, false);

        //while(!t1.isReady()){
        if(isStopRequested()){
            return;
        }
        runtime.reset();
        waitForStart();

        //robot.forward(0.25, 0.6);


        telemetry.addData("angle", robot.getHeading());
        telemetry.update();
        sleep(5000);
        //robot.turn(0.3,Math.PI/2, false);
        telemetry.addData("angle", robot.getHeading());
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("Left deadwheel", 5.08*Math.PI*robot.lB.getCurrentPosition()/8192);
            telemetry.addData("right deadwheel", 5.08*Math.PI*robot.rF.getCurrentPosition()/8192);
            telemetry.addData("Middle  deadwheel", 66.17647058823529*5.08*Math.PI*robot.lF.getCurrentPosition()/8192);
            telemetry.addData("Position", "X: "+robot.odo.getX()+"Y: "+robot.odo.getY()+"Angle: "+robot.odo.getAngle());
            telemetry.update();
        }
    }
}
