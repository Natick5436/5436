package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Mark_4{
    DcMotor left, right, middle, liftL, liftR;
    Servo hook, grab;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    CRServo flyR, flyL, extension;
    BNO055IMU imu;
    boolean skystone = false;

    //wheel Diameter in cm
    final double wheelDiameter = 10.16;
    //wheel Circumference in meters
    final double wheelCirc = wheelDiameter * Math.PI/100;
    //ticks per full revolution of the motor axle
    final double ticksPerMotorRev = 1120.0;
    final double motorGearRatio = 1.0;
    //ticks per full revolution of the wheel
    final double ticksPer = motorGearRatio*ticksPerMotorRev;

    double odometryX;
    double odometryY;
    double odometryAngle, initialAngle;
    double lastEncoderR, lastEncoderL;
    double currentEncoderL, currentEncoderR;
    DcMotor deadWheel;
    final double ticksPerOdometryWheel = 1430;
    //diameter in cm
    final double odometryWheelDiameter = 7.62;
    final double odometryWheelCirc = odometryWheelDiameter * Math.PI / 100;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    final double angleAccuracy = 0.005;
    final double distanceAccuracy = 0.01;
    View relativeLayout;

    Orientation angles;

    LinearOpMode ln;
    public Mark_4(LinearOpMode linear, HardwareMap hardwareMap){
        ln = linear;
    }

    public void initialize(HardwareMap hardwareMap, double startX, double startY, double startAngle) {
        odometryX = startX;
        odometryY = startY;
        odometryAngle = ACMath.toStandardAngle(startAngle);
        initialAngle = ACMath.toStandardAngle(startAngle);

        left = hardwareMap.dcMotor.get("l");
        right = hardwareMap.dcMotor.get("r");
        middle = hardwareMap.dcMotor.get("m");
        extension = hardwareMap.crservo.get("extension");

        hook = hardwareMap.servo.get("hook");
        flyR = hardwareMap.crservo.get("flyR");
        flyL = hardwareMap.crservo.get("flyL");
        grab = hardwareMap.servo.get("grab");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");

        sensorColor = hardwareMap.get(ColorSensor.class, "skySensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "skySensor");

        deadWheel = hardwareMap.dcMotor.get("deadWheel");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hook.setPosition(0.5);
        grab.setPosition(0.0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sensorColor = hardwareMap.get(ColorSensor.class, "skySensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "skySensor");
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        while(imu.isGyroCalibrated()){
            if(ln.isStopRequested()){
                return;
            }
        }
        ln.telemetry.addData("Ready","Ready");
        ln.telemetry.update();
    }
    //place this in at the end of every loop to update the values.
    public void updateLinearOdometryData(){
        odometryAngle = ACMath.toStandardAngle(getHeading() + initialAngle);
        currentEncoderL = left.getCurrentPosition();
        currentEncoderR = right.getCurrentPosition();
        double deltaTickL = currentEncoderL - lastEncoderL;
        double deltaTickR = currentEncoderR - lastEncoderR;
        double dL = wheelCirc * deltaTickL / ticksPer;
        double dR = wheelCirc * deltaTickR / ticksPer;
        double dM = (dL + dR) / 2;
        odometryX = odometryX + (dM * Math.cos(odometryAngle));
        odometryY = odometryY + (dM * Math.sin(odometryAngle));
        lastEncoderL = currentEncoderL;
        lastEncoderR = currentEncoderR;

    }
    public void setOdometryPosition(double x, double y){
        odometryX = x;
        odometryY = y;
    }

    public void turn(double power, double targetAngle){
        odometryAngle = ACMath.toStandardAngle(getHeading() + initialAngle);
        double targetAngleDelta = targetAngle - odometryAngle;
        /*if(ACMath.compassAngleShorter(targetAngle, odometryAngle)) {
            targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(odometryAngle);
        }else{
            targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(odometryAngle);
        }*/
        double targetAngleAbs = Math.abs(targetAngleDelta);
        boolean sw = false;
        if (targetAngleDelta < 0)
            sw = true;
        if (targetAngleDelta > 0)
            sw = false;
        int decreaseRate = 0;
        while (targetAngleAbs >= angleAccuracy){
            odometryAngle = ACMath.toStandardAngle(getHeading() + initialAngle);
            if (targetAngleDelta > 0){
                right.setPower(-power / decreaseRate);
                left.setPower(power / decreaseRate );
                if(sw == false){
                    decreaseRate++;
                }
                sw = true;
            }
            if (targetAngleDelta < 0){
                left.setPower(-power /  decreaseRate);
                right.setPower(power / decreaseRate);
                if(sw == true){
                    decreaseRate++;
                }
                sw = false;
            }
            targetAngleDelta = targetAngle - odometryAngle;
            targetAngleAbs = Math.abs(targetAngleDelta);
            if(ln.isStopRequested())return;
            ln.telemetry.addData("targetAngleDelta", targetAngleDelta);
            ln.telemetry.addData("odometryAngle", odometryAngle);
            ln.telemetry.addData("targetAngle", targetAngle);
            ln.telemetry.update();
        }
        right.setPower(0);
        left.setPower(0);
    }
    public void forwardMeters(double meters, double power){
        double targetTickDelta = (meters / wheelCirc)*ticksPer;
        double targetTickR = targetTickDelta+right.getCurrentPosition();
        double targetTickL = targetTickDelta+left.getCurrentPosition();
        double deltatickR;
        double deltatickL;
        while (Math.abs(right.getCurrentPosition()-targetTickR) >= (distanceAccuracy/wheelCirc)*ticksPer || Math.abs(left.getCurrentPosition()-targetTickL) >= (distanceAccuracy/wheelCirc)*ticksPer) {
            ln.telemetry.addData("left encoder", left.getCurrentPosition());
            ln.telemetry.addData("right encoder", right.getCurrentPosition());
            ln.telemetry.addData("targetTickL", targetTickL);
            ln.telemetry.addData("targetTickR", targetTickR);
            ln.telemetry.update();
            deltatickL = targetTickL - left.getCurrentPosition();
            deltatickR = targetTickR - right.getCurrentPosition();
            if(right.getCurrentPosition() != targetTickR) {
                right.setPower(power * (Math.abs(deltatickR) / deltatickR));
            }else{
                right.setPower(0);
            }
            if (left.getCurrentPosition() != targetTickL) {
                left.setPower(power * (Math.abs(deltatickL) / deltatickL));
            }else{
                left.setPower(0);
            }
            if(ln.isStopRequested())return;
            updateLinearOdometryData();
        }
        left.setPower(0);
        right.setPower(0);
    }

    public void  goToDeltaPosition(double meters, double targetAngle, double power){
        turn(power, targetAngle);
        forwardMeters(meters, power);
    }

    public void goToAbsolutePosition(double x, double y, double power){
        double deltaX = x-odometryX;
        double deltaY = y-odometryY;
        double angle = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaY, deltaX);
        goToDeltaPosition(distance, angle, power);
    }

    public double getHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.getPosition();
        return Math.toRadians(angles.firstAngle);
    }
    public boolean isSkystone(){
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        if (sensorColor.red() > 85 && sensorColor.green() > 65){
            skystone = false;
        }else if (sensorColor.red() < 40 && sensorColor.green() < 45){
            skystone = true;
        }else{
            skystone = false;
        }
        return skystone;
    }
}