package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Straight Forward Auto", group = "Autonomous")
@Disabled
public class Straight_Forward_Auto extends LinearOpMode {

        Mark_5 robot = new Mark_5(this);

    @Override public void runOpMode(){

        robot.initialize(hardwareMap,0,0,0, false);
        waitForStart();
        robot.arm.setPower(-0.6);
        sleep(800);
        robot.arm.setPower(0);
        robot.extensionL.setPosition(0.84);
        robot.extensionR.setPosition(0.85);
        robot.forward(.25,.67);

    }

}
