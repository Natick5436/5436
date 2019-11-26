package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Noland_Is_A_Bad_Teacher_Autonomus extends LinearOpMode {

        Mark_5 robot = new Mark_5(this);

    @Override public void runOpMode(){

        robot.initialize(hardwareMap,0,0,0);
        waitForStart();
        robot.forward(1,.67);

    }

}
