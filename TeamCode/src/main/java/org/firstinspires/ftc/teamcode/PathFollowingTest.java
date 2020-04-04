package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Execution;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Point;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.ServoMove;

import java.util.ArrayList;

@Autonomous(name = "Figure 8", group = "Autonomous")
public class PathFollowingTest extends LinearOpMode {
    Mark_6 robot = new Mark_6(this);


    public void runOpMode(){
        robot.initialize(hardwareMap, 0, 0, 0, false);
        ArrayList<Execution> path = new ArrayList<Execution>();
        path.add(new Point(0.5, 0.5));
        path.add(new Point(0, 1));
        path.add(new Point(-0.5, 1.5));
        path.add(new Point(0, 2));
        path.add(new ServoMove(robot.foundation, robot.FOUNDATION_CLOSE, 5000, ServoMove.MovementType.OVER_TIME, this));
        path.add(new Point(0.5, 1.5));
        path.add(new Point(0, 1));
        path.add(new Point(-0.5, 0.5));
        path.add(new Point(0, 0));

        waitForStart();
        robot.goToDeltaCurvePosition(0.5, 0.25, 0.25, true);
/*
        for(Execution step : path){
            if(step instanceof Point){
                robot.goToAbsoluteCurvePosition(0.4, ((Point)step).getX(), ((Point)step).getY(), false);
            }
            step.run();
        }
 */
    }
}
