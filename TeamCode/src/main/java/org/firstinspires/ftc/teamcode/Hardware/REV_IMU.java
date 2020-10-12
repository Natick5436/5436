package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.AngleTracker;

public class REV_IMU implements AngleTracker {
    private BNO055IMU imu;
    private LinearOpMode ln;
    private double initialAngle;
    private int angleAxis;
    public REV_IMU(LinearOpMode linearOpMode, String initName, int angleAxis, double initialAngle){
        ln = linearOpMode;
        this.initialAngle = initialAngle;
        this.angleAxis = angleAxis;
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = ln.hardwareMap.get(BNO055IMU.class, initName);
        imu.initialize(imuParameters);

        while(imu.isGyroCalibrated()){
            if(ln.isStopRequested()){
                return;
            }
            if(ln.isStarted()){
                return;
            }
        }
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.getPosition();
        switch (angleAxis){
            case 1:{
                return ACMath.toStandardAngle(Math.toRadians(angles.firstAngle) + initialAngle);
            }
            case 2:{
                return ACMath.toStandardAngle(Math.toRadians(angles.secondAngle) + initialAngle);
            }
            case 3: {
                return ACMath.toStandardAngle(Math.toRadians(angles.thirdAngle) + initialAngle);
            }
        }
        return ACMath.toStandardAngle(Math.toRadians(angles.firstAngle) + initialAngle);
    }
    public void update(){

    }
}
