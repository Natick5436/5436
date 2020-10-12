package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;

import java.util.ArrayList;

@TeleOp(name = "WheelSpeeds", group = "TeleOp")
public class WheelSpeedTest extends LinearOpMode {
    Mecanum_Drive robot;
    public void runOpMode() throws InterruptedException{
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 9");
        robot = new Mecanum_Drive(hardwareMap.dcMotor.get("lF"),hardwareMap.dcMotor.get("lB"),hardwareMap.dcMotor.get("rF"),hardwareMap.dcMotor.get("rB"), DcMotor.RunMode.RUN_USING_ENCODER, 0.05, 0.336, 0.39625, 185);
/*
        ArrayList<Double> speeds = new ArrayList<Double>();

        double lastEncoder = robot.rB.getCurrentPosition();
        double lastTime = System.currentTimeMillis();
        while(!isStarted()){
            double currentTime = System.currentTimeMillis();
            double currentEncoder = robot.rB.getCurrentPosition();
            robot.rB.setPower(1);
            double rpmAtTime = ((currentEncoder-lastEncoder)/((currentTime-lastTime)/60000))/383.6;
            speeds.add(new Double(rpmAtTime));
            telemetry.addData("RPM", rpmAtTime);
            telemetry.update();
            lastTime = currentTime;
            lastEncoder = currentEncoder;
        }
        robot.stopDrive();
        waitForStart();
        long average;
        long sum = Long.valueOf(0);
        for(int i=0; i<speeds.size(); i++){
            sum += speeds.get(i).doubleValue();
        }
        average = sum/speeds.size();
        while (opModeIsActive()){
            telemetry.addData("Avg Speed", average);
            telemetry.addData("sum", sum);
            telemetry.addData("Size", speeds.size());
            telemetry.update();
        }*/
        waitForStart();

        while(opModeIsActive()){
            robot.inverseKinematics(new double[]{0.5, 1.071*0.5, 0});
        }
        robot.stopDrive();
    }
}
