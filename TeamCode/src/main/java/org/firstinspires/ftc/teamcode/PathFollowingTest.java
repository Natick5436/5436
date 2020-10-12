package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.Math.PurePursuitPath;
import org.firstinspires.ftc.teamcode.Math.Waypoint;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Odometry;
import android.graphics.*;
import android.media.Image;
import android.media.MediaCodec;
import android.widget.ImageView;

import java.io.File;
import java.util.Arrays;


@Autonomous(name = "Path Following", group = "Autonomous")
public class PathFollowingTest extends LinearOpMode {
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    BNO055IMU imu;
    double initialAngle = Math.PI/2;
    Odometry odo;
    public void runOpMode(){
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 9");
        Mecanum_Drive robot = new Mecanum_Drive(hardwareMap.dcMotor.get("lF"),hardwareMap.dcMotor.get("lB"),hardwareMap.dcMotor.get("rF"),hardwareMap.dcMotor.get("rB"), DcMotor.RunMode.RUN_USING_ENCODER, 0.05, 0.336, 0.39625, 185);
        double LENGTH = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
        double naturalMiddleMovementPerRadian = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        odo = new Odometry(this, new DeadWheel(hardwareMap.dcMotor.get("misc")), new DeadWheel(hardwareMap.dcMotor.get("intakeR")), new DeadWheel(hardwareMap.dcMotor.get("intakeL")), Mark_6.wheelDiameter, Mark_6.ticksPerMotorRev, Mark_6.motorGearRatio, LENGTH, naturalMiddleMovementPerRadian, 0, 0, initialAngle, -1, 1, -1);
        odo.start();
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
        telemetry.addData("Voltage: ", voltageSensor.getVoltage());
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

        //Follow path forward
        double maintainedAngle = 0;
        Waypoint[] points = new Waypoint[]{new Waypoint(0, 0), new Waypoint(0, 1), new Waypoint(1, 1)};       //Meters

        PurePursuitPath path = new PurePursuitPath(points);
        path.mapTimes(0.7, robot);     //Meters per seconds
        for(int i=0; i<path.pathWaypoints.length; i++){
            telemetry.addData("Point "+i+" ", path.pathWaypoints[i].getTime());
        }
        telemetry.update();
        double m = 3*Math.PI/4; //Max Angular velocity to correct angle
        double s = Math.PI/2; //Max displacement angle before angle correction is saturated
        // Angle correction code m*Range.clip((angleCorrection)/s, -1, 1))

        double positionError = 0.3; //How much the odometry can be off by before it stops the path and recalculates
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < path.getTotalPathTime()*1000.0){
            double timeSeconds = (System.currentTimeMillis()-startTime)/1000.0;
            double angleCorrection;
            if(ACMath.compassAngleShorter(maintainedAngle+getHeading(), path.angleByTime(timeSeconds))){
                angleCorrection = ACMath.toCompassAngle(path.angleByTime(timeSeconds)) - ACMath.toCompassAngle(maintainedAngle+getHeading());
            }else{
                angleCorrection =  ACMath.toStandardAngle(path.angleByTime(timeSeconds)) - ACMath.toStandardAngle(maintainedAngle+getHeading());
            }
            double[] speeds = robot.inverseKinematics(new double[]{path.linearVelocityOfTime(timeSeconds)*Math.cos(path.angleByTime(timeSeconds)-getHeading()), path.linearVelocityOfTime(timeSeconds)*Math.sin(path.angleByTime(timeSeconds)-getHeading()), (path.anglePrimeByTime(timeSeconds) + m*Range.clip((angleCorrection)/s, -1, 1))});
            telemetry.addData("Position: ", "X: "+path.xOfTime(timeSeconds)+" Y: "+path.yOfTime(timeSeconds));
            telemetry.addData("Odo Position: ", "X: "+odo.getX()+" Y: "+odo.getY());
            telemetry.addData("Wheel speeds", robot.rB.getPower());
            telemetry.update();
            if(isStopRequested()){
                break; 
            }
            if(Math.hypot(path.xOfTime(timeSeconds)-odo.getX(), path.yOfTime(timeSeconds)-odo.getY()) > positionError){
                break;
            }
        }
        /*
        //Follow path angle independent
        double targetAngle = 0;
        double desiredVoltage = 12.0;
        Waypoint[] points = new Waypoint[]{new Waypoint(0, 0), new Waypoint(0, 0.5), new Waypoint(0.75, 0.5), new Waypoint(1, 0.25), new Waypoint(0.5, 0.25)};       //Meters

        PurePursuitPath path = new PurePursuitPath(points);
        path.mapTimes(0.35);     //Meters per seconds
        double m = Math.PI;
        double s = Math.PI/2;
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < path.getTotalPathTime()*1000.0+2000){
            double timeSeconds = (System.currentTimeMillis()-startTime)/1000.0;
            double angleCorrection;
            if(ACMath.compassAngleShorter(getHeading(), targetAngle)){
                angleCorrection = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(getHeading());
            }else{
                angleCorrection =  ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(getHeading());
            }
            double[] speeds = robot.inverseKinematics(new double[]{desiredVoltage*path.linearVelocityOfTime(timeSeconds)*Math.cos(path.angleByTime(timeSeconds)-getHeading())/voltageSensor.getVoltage(), desiredVoltage*path.linearVelocityOfTime(timeSeconds)*Math.sin(path.angleByTime(timeSeconds)-getHeading())/voltageSensor.getVoltage(), (m*(angleCorrection)/s)/voltageSensor.getVoltage()});
            telemetry.addData("speeds", Arrays.toString(speeds));
            telemetry.addData("TotalPathTime: ", path.getTotalPathTime()*1000.0);
            telemetry.addData("Last Point time: ", path.pathWaypoints[path.pathWaypoints.length-1].getTime()*1000.0);
            telemetry.addData("CurrentTime: ", System.currentTimeMillis()-startTime);
            telemetry.update();
            if(isStopRequested()){
                break;
            }
        }*/
        robot.setPowerAll(new double[]{-robot.lF.getPower(), -robot.rF.getPower(), -robot.lB.getPower(), -robot.rB.getPower()});
        robot.stopDrive();
        sleep(10000);
        telemetry.addData("X: ", odo.getX());
        telemetry.addData("Y: ", odo.getY());
        telemetry.addData("Angle: ", odo.getAngle());
        telemetry.update();
        double targetAngle = Math.atan2(-odo.getY(), -odo.getX());
        while(Math.abs(targetAngle-odo.getAngle())>0.05){
            robot.inverseKinematics(new double[]{0, 0, 2*(targetAngle-odo.getAngle())/Math.PI + 0.25*Math.abs(targetAngle-odo.getAngle())/(targetAngle-odo.getAngle())});
            if (isStopRequested()){
                break;
            }
        }
        robot.stopDrive();
        while(opModeIsActive()){
            telemetry.addData("X: ", odo.getX());
            telemetry.addData("Y: ", odo.getY());
            telemetry.addData("Angle: ", odo.getAngle());
            telemetry.addData("Left: ", odo.left.getCurrentPosition());
            telemetry.addData("Right: ", odo.right.getCurrentPosition());
            telemetry.addData("Mid: ", odo.middle.getCurrentPosition());
            telemetry.update();
        }
    }
    //Misc methods
    public double getHeading(){
        return odo.getAngle();
    }
}
