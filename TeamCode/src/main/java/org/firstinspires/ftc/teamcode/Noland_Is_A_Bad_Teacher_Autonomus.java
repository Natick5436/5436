package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Noland Is A Bad Teacher", group = "Autonomous")
public class Noland_Is_A_Bad_Teacher_Autonomus extends LinearOpMode {

        Mark_5 robot = new Mark_5(this);

    @Override public void runOpMode(){

        robot.initialize(hardwareMap,0,0,0);
        waitForStart();
        robot.forward(.25,.67);

    }

}
