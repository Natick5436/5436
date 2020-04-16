package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Odometry;

import java.io.File;
import java.util.Arrays;


@Autonomous(name = "Path Following", group = "Autonomous")
public class PathFollowingTest extends LinearOpMode {
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    BNO055IMU imu;
    public void runOpMode(){
        Mecanum_Drive robot = new Mecanum_Drive(hardwareMap.dcMotor.get("lF"),hardwareMap.dcMotor.get("lB"),hardwareMap.dcMotor.get("rF"),hardwareMap.dcMotor.get("rB"), 0.1, 0.336, 0.39625, 217.5);
        double LENGTH = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
        double naturalMiddleMovementPerRadian = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        Odometry odo = new Odometry(this, new DeadWheel(hardwareMap.dcMotor.get("misc")), new DeadWheel(hardwareMap.dcMotor.get("intakeR")), new DeadWheel(hardwareMap.dcMotor.get("intakeL")), Mark_6.wheelDiameter, Mark_6.ticksPerMotorRev, Mark_6.motorGearRatio, LENGTH, naturalMiddleMovementPerRadian, 0, 0, 0, -1, 1, 1);
        //***IMU init***
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        while(imu.isGyroCalibrated()){
            if(isStopRequested()){
                return;
            }
            if(isStarted()){
                return;
            }
        }
        telemetry.addData("Status", "Ready");
        telemetry.addData("inverseKinematicsArray", Arrays.toString(robot.inverseKinematicsMatrix));
        telemetry.update();

        waitForStart();
        /*double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < 5000){
            double[] speeds = robot.inverseKinematics(new double[]{1, 0, 0});
            telemetry.addData("speeds", Arrays.toString(speeds));
        }*/
        /*
        while(ACMath.toCompassAngle(getHeading())<Math.PI){
            double[] speeds = robot.inverseKinematics(new double[]{0.5*Math.cos(getHeading()), -0.5*Math.sin(getHeading()), 0.5* 3*Math.PI/4});
            if(isStopRequested()){
                return;
            }
            telemetry.addData("Heading: ", getHeading());
            telemetry.addData("Speeds", Arrays.toString(speeds));
            telemetry.addData("lF", robot.lF.getPower());
            telemetry.addData("lB", robot.lB.getPower());
            telemetry.addData("rF", robot.rF.getPower());
            telemetry.addData("rB", robot.rB.getPower());
            telemetry.update();
        }*/
        double a = 0.5;
        double p = 0.25;
        /*Spiral path follow
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < 7000){
            double timeSeconds = (System.currentTimeMillis()-startTime)/1000.0;
            //The goal of this inverse kinematics is to have the robot follow a cosine wave with amplitude 0.25m and a period of 1 meter
            double[] speeds = robot.inverseKinematics(new double[]{a*Math.sqrt(Math.pow(p*timeSeconds,2)+1), 0, (Math.pow(p*timeSeconds, 2)+2)/(Math.pow(p*timeSeconds,2)+1)});
            telemetry.addData("speeds", Arrays.toString(speeds));
            telemetry.update();
            if(isStopRequested()){
                return;
            }
        }*/
        /*
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < 28000){
            double timeSeconds = (System.currentTimeMillis()-startTime)/1000.0;
            //The goal of this inverse kinematics is to have the robot follow a cosine wave with amplitude 0.25m and a period of 1 meter
            double[] speeds = robot.inverseKinematics(new double[]{a*p*Math.sqrt(Math.pow(Math.cos(p*timeSeconds),2)+Math.pow(Math.cos(p*timeSeconds),4)-2*Math.pow(Math.sin(p*timeSeconds),2)*Math.pow(Math.cos(p*timeSeconds),2)+Math.pow(Math.sin(p*timeSeconds),4)),
                    0, p*(-3*Math.sin(p*timeSeconds)*Math.pow(Math.cos(p*timeSeconds),2)-Math.pow(Math.sin(p*timeSeconds),3))/(Math.pow(Math.cos(p*timeSeconds),2)+Math.pow(Math.cos(p*timeSeconds),4)-2*Math.pow(Math.sin(p*timeSeconds),2)*Math.pow(Math.cos(p*timeSeconds),2)+Math.pow(Math.sin(p*timeSeconds),4))});
            telemetry.addData("speeds", Arrays.toString(speeds));
            telemetry.update();
            if(isStopRequested()){
                return;
            }
        }*/

        robot.setPowerAll(new double[]{-robot.lF.getPower(), -robot.rF.getPower(), -robot.lB.getPower(), -robot.rB.getPower()});
        robot.stopDrive();
    }
    //Misc methods
    public double getHeading(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.getPosition();
        return ACMath.toStandardAngle(Math.toRadians(angles.firstAngle));
    }
}
