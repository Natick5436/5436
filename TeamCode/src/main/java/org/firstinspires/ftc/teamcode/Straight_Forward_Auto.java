package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Straight Forward Auto", group = "Autonomous")
public class Straight_Forward_Auto extends LinearOpMode {

        Mark_5 robot = new Mark_5(this);

    @Override public void runOpMode(){

        robot.initialize(hardwareMap,0,0,0, false);
        waitForStart();
        robot.forward(.25,.67);

    }

}
