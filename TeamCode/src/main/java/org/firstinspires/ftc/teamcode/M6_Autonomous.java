package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous(name = "Mark 6 Calibration", group = "Autonomous")
public class M6_Autonomous extends LinearOpMode {

    Mark_6 robot = new Mark_6(this);
    ElapsedTime runtime = new ElapsedTime();

    final double SKYSTONE_ACCURACY = 0.5;
    final double MIDDLE_OF_SCREEN = -11;
    int stonePosition;

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    //Measures certain physical constants of the robot and stores them in files
    @Override
    public void runOpMode() throws InterruptedException {
        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        robot.initialize(hardwareMap, 0, 0, 0, false);
        runtime.reset();
        waitForStart();
        robot.turn(0.5, Math.PI/2);
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(((0.0508*Math.PI*robot.lB.getCurrentPosition()/8192) + (0.0508*Math.PI*robot.rF.getCurrentPosition()/8192))/robot.getHeading()));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(-(robot.middleDeadWheelCorrection*0.0508*Math.PI*robot.lF.getCurrentPosition()/8192)/robot.getHeading()));
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
