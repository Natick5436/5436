package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name = "PointToPoint", group = "Autonomous")
public class PointToPoint_Auto extends LinearOpMode {
    Gilgamesh gilgamesh = new Gilgamesh(this);
    @Override
    public void runOpMode() {
        gilgamesh.initialize(hardwareMap, 0, 0, 0);
        sleep(1000);
        waitForStart();
       // gilgamesh.goToPointArch(1, 0.5);
        //sleep(1000);
        gilgamesh.goToPointLine(0.5, 0.5);
        sleep(1000);
        gilgamesh.goToPointLine(0, 0);
        sleep(1000);
    }
}

