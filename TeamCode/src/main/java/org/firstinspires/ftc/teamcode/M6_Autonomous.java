package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mark 6 Calibration", group = "Autonomous")
public class M6_Autonomous extends LinearOpMode {

    Mark_6 robot = new Mark_6(this);
    ElapsedTime runtime = new ElapsedTime();

    final double metersPerMm = 0.001;

    final int X = 0;
    final int Y = 1;
    final int Z = 2;

    final double SKYSTONE_ACCURACY = 0.5;
    final double MIDDLE_OF_SCREEN = -11;
    int stonePosition;


    @Override
    public void runOpMode() throws InterruptedException {

        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        robot.initialize(hardwareMap, 0, 0, 0, false);
        runtime.reset();
        robot.odo.start();
        waitForStart();
        telemetry.addData("angle", robot.getHeading());
        telemetry.update();
        robot.forward(0.5, 0.5);
      //  robot.turn(0.3,Math.PI/2, false);
        telemetry.addData("angle", robot.getHeading());
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("Left deadwheel", -5.08*Math.PI*robot.lB.getCurrentPosition()/8192);
            telemetry.addData("right deadwheel", 5.08*Math.PI*robot.rF.getCurrentPosition()/8192);
            telemetry.addData("Middle  deadwheel", -66.17647058823529*5.08*Math.PI*robot.lF.getCurrentPosition()/8192);
            telemetry.addData("Position", "X: "+robot.odo.getX()+"Y: "+robot.odo.getY()+"Angle: "+robot.odo.getAngle());
            telemetry.addData("Length: ", ((0.0508*Math.PI*robot.lB.getCurrentPosition()/8192) + (0.0508*Math.PI*robot.rF.getCurrentPosition()/8192))/robot.getHeading());
            telemetry.addData("Middle to angle ratio: ", -(66.17647058823529*0.0508*Math.PI*robot.lF.getCurrentPosition()/8192)/robot.getHeading());
            telemetry.update();
        }
    }
}
