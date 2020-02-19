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
        robot.initialize(hardwareMap, 0, 0, 0, true);
        runtime.reset();
        waitForStart();
        robot.odo.start();
        double startTime = System.currentTimeMillis();
        double currentTime = System.currentTimeMillis();
        while(Math.abs(currentTime-startTime) < 1000 && opModeIsActive()){
            robot.updatePDVelocityL(-0.5);
            robot.updatePDVelocityR(0.5);
            currentTime = System.currentTimeMillis();
        }
        robot.stopDrive();
        Thread.sleep(1000);
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(((robot.wheelCirc*robot.odo.right.getCurrentPosition()/robot.ticksPer) - (robot.wheelCirc*robot.odo.left.getCurrentPosition()/robot.ticksPer))/robot.getHeading()));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf((robot.middleDeadWheelCorrection*robot.wheelCirc*robot.odo.middle.getCurrentPosition()/robot.ticksPer)/robot.getHeading()));
        while(opModeIsActive()){
            telemetry.addData("Left deadwheel", robot.wheelCirc*robot.odo.left.getCurrentPosition()/robot.ticksPer);
            telemetry.addData("right deadwheel", robot.wheelCirc*robot.odo.right.getCurrentPosition()/robot.ticksPer);
            telemetry.addData("Middle  deadwheel", robot.middleDeadWheelCorrection*robot.wheelCirc*robot.odo.middle.getCurrentPosition()/robot.ticksPer);
            telemetry.addData("Length: ", ((robot.wheelCirc*robot.odo.right.getCurrentPosition()/robot.ticksPer) - (robot.wheelCirc*robot.odo.left.getCurrentPosition()/robot.ticksPer))/robot.getHeading());
            telemetry.addData("Middle to angle ratio: ", (robot.middleDeadWheelCorrection*robot.wheelCirc*robot.odo.middle.getCurrentPosition()/robot.ticksPer)/robot.getHeading());
            telemetry.addData("Heading",robot.getHeading());
            telemetry.update();
        }
    }
}
