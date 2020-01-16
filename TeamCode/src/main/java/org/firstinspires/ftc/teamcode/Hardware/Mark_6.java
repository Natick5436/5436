package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Math.ACMath;

public class Mark_6 {
    private Mark_5.Status robotStatus;
    public void setStatus(Mark_5.Status s){
        robotStatus = s;
    }
    public Mark_5.Status getStatus(){
        return robotStatus;
    }

    public DcMotor lF, lB, rF, rB;
    public REVEncoder rDW;

    BNO055IMU imu;
    Orientation angles;

    //in meters or radians respectively unless specified
    public double odometryX;
    public double odometryY;
    public double odometryAngle, initialAngle;
    double lastEncoderR, lastEncoderL;
    double currentEncoderL, currentEncoderR;

    //wheel Diameter in cm
    final double wheelDiameter = 10.0;
    //wheel Circumference in meters
    final double wheelCirc = wheelDiameter * Math.PI/100;
    //ticks per full revolution of the motor axle
    final double ticksPerMotorRev = 383.6;
    final double motorGearRatio = 0.5;
    //ticks per full revolution of the wheel
    final double ticksPer = motorGearRatio*ticksPerMotorRev;

    final double angleAccuracy = 0.005;
    final double distanceAccuracy = 0.01;

    LinearOpMode ln;
    public Mark_6(LinearOpMode linear) {
        ln = linear;
    }
    public void initialize(HardwareMap hardwareMap, double startX, double startY, double startAngle, boolean teleOpInit) {
        //***Main robot init***
        lF = hardwareMap.dcMotor.get("lF");
        lB = hardwareMap.dcMotor.get("lB");
        rF = hardwareMap.dcMotor.get("rF");
        rB = hardwareMap.dcMotor.get("rB");

        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);

        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rDW = new REVEncoder("cA", "cB", ln);

        //***Odometry init***
        odometryX = startX;
        odometryY = startY;
        odometryAngle = ACMath.toStandardAngle(startAngle);
        initialAngle = ACMath.toStandardAngle(startAngle);

        //***IMU init***
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        while(imu.isGyroCalibrated()){
            if(ln.isStopRequested()){
                return;
            }
            if(ln.isStarted()){
                return;
            }
        }

        ln.telemetry.addData("Status", "Ready");
        ln.telemetry.update();
    }

    //Odometry methods
    public void updateLinearOdometryData(){
        odometryAngle = getHeading();
        currentEncoderL = lF.getCurrentPosition();
        currentEncoderR = rF.getCurrentPosition();
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

    //Movement methods
    double startAngle;
    final double percentCorrection = 0;
    final double maxCorrectionAngle = Math.PI/12;
    public void forward(double power){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if(getStatus() != Mark_5.Status.DRIVING){
            startAngle = getHeading();
        }
        this.setStatus(Mark_5.Status.DRIVING);
        double turnOffset;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }else{
            turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }
        lF.setPower(power+turnOffset);
        lB.setPower(power+turnOffset);
        rF.setPower(power-turnOffset);
        rB.setPower(power-turnOffset);
        updateLinearOdometryData();
    }
    //this function only takes inputs from the front wheels of the robot
    public void forward(double power, double meters){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if(getStatus() != Mark_5.Status.DRIVING){
            startAngle = getHeading();
        }
        double targetTickDelta = (meters / wheelCirc)*ticksPer;
        double targetTickR = targetTickDelta+rF.getCurrentPosition();
        double targetTickL = targetTickDelta+lF.getCurrentPosition();
        double deltatickR;
        double deltatickL;
        this.setStatus(Mark_5.Status.DRIVING);
        double turnOffset;
        //(Math.abs(rF.getCurrentPosition()-targetTickR) + Math.abs(lF.getCurrentPosition()-targetTickL))/2 >= (distanceAccuracy/wheelCirc)*ticksPer
        while ((Math.abs(rF.getCurrentPosition()-targetTickR) + Math.abs(lF.getCurrentPosition()-targetTickL))/2 >= (distanceAccuracy/wheelCirc)*ticksPer) {
            if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }else{
                turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }
            deltatickL = targetTickL - lF.getCurrentPosition();
            deltatickR = targetTickR - rF.getCurrentPosition();
            if(rF.getCurrentPosition() != targetTickR) {
                rF.setPower((power * (Math.abs(deltatickR) / deltatickR))-turnOffset);
                rB.setPower((power * (Math.abs(deltatickR) / deltatickR))-turnOffset);
            }else{
                rF.setPower(0);
                rB.setPower(0);
            }
            if (lF.getCurrentPosition() != targetTickL) {
                lF.setPower((power * (Math.abs(deltatickL) / deltatickL))+turnOffset);
                lB.setPower((power * (Math.abs(deltatickL) / deltatickL))+turnOffset);
            }else{
                lF.setPower(0);
                lB.setPower(0);
            }
            ln.telemetry.addData("left encoder", lF.getCurrentPosition());
            ln.telemetry.addData("right encoder", rF.getCurrentPosition());
            ln.telemetry.addData("targetTickL", targetTickL);
            ln.telemetry.addData("targetTickR", targetTickR);
            ln.telemetry.update();
            if(ln.isStopRequested())return;
            updateLinearOdometryData();
        }
        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }
    public void turn(double power){
        this.setStatus(Mark_5.Status.TURNING);
        rF.setPower(power);
        rB.setPower(power);
        lF.setPower(-power);
        lB.setPower(-power);
    }
    public void turn(double power, double targetAngle){
        odometryAngle = getHeading();
        double angle = odometryAngle;
        double targetAngleDelta;
        boolean useCompassAngle = ACMath.compassAngleShorter(targetAngle, angle);
        if(useCompassAngle) {
            targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
        }else{
            targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
        }
        double targetAngleAbs = Math.abs(targetAngleDelta);
        boolean sw = false;
        if (targetAngleDelta > 0)
            sw = true;
        if (targetAngleDelta < 0)
            sw = false;
        int decreaseRate = 0;
        while (targetAngleAbs >= angleAccuracy){
            //If change is negative, robot turns in negative direction
            if (targetAngleDelta < 0){
                //Turn right
                rF.setPower(-power / decreaseRate);
                rB.setPower(-power / decreaseRate);
                lF.setPower(power / decreaseRate);
                lB.setPower(power / decreaseRate);
                if(!sw){
                    decreaseRate++;
                }
                sw = true;
            }
            if (targetAngleDelta > 0){
                //Turn left
                lF.setPower(-power /  decreaseRate);
                lB.setPower(-power /  decreaseRate);
                rF.setPower(power / decreaseRate);
                rB.setPower(power / decreaseRate);
                if(sw){
                    decreaseRate++;
                }
                sw = false;
            }
            odometryAngle = getHeading();
            angle = odometryAngle;
            if(useCompassAngle) {
                targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
            }else{
                targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
            }
            targetAngleAbs = Math.abs(targetAngleDelta);
            if(ln.isStopRequested())return;
            ln.telemetry.addData("targetAngleDelta", targetAngleDelta);
            ln.telemetry.addData("odometryAngle", angle);
            ln.telemetry.addData("targetAngle", targetAngle);
            ln.telemetry.addData("Compass Angle: ", useCompassAngle);
            ln.telemetry.update();
        }
        rF.setPower(0);
        rB.setPower(0);
        lF.setPower(0);
        lB.setPower(0);
    }
    public void turn(double power, double targetAngle,boolean override){
        odometryAngle = getHeading();
        double angle = odometryAngle;
        double targetAngleDelta;
        if(override) {
            targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
        }else{
            targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
        }
        double targetAngleAbs = Math.abs(targetAngleDelta);
        boolean sw = false;
        if (targetAngleDelta > 0)
            sw = true;
        if (targetAngleDelta < 0)
            sw = false;
        int decreaseRate = 0;
        while (targetAngleAbs >= angleAccuracy){
            if (targetAngleDelta < 0){
                rF.setPower(-power / decreaseRate);
                rB.setPower(-power / decreaseRate);
                lF.setPower(power / decreaseRate);
                lB.setPower(power / decreaseRate);
                if(!sw){
                    decreaseRate++;
                }
                sw = true;
            }
            if (targetAngleDelta > 0){
                lF.setPower(-power /  decreaseRate);
                lB.setPower(-power /  decreaseRate);
                rF.setPower(power / decreaseRate);
                rB.setPower(power / decreaseRate);
                if(sw){
                    decreaseRate++;
                }
                sw = false;
            }
            odometryAngle = getHeading();
            angle = odometryAngle;
            if(override) {
                targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
            }else{
                targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
            }
            targetAngleAbs = Math.abs(targetAngleDelta);
            if(ln.isStopRequested())return;
            ln.telemetry.addData("targetAngleDelta", targetAngleDelta);
            ln.telemetry.addData("odometryAngle", angle);
            ln.telemetry.addData("targetAngle", targetAngle);
            ln.telemetry.addData("abs",targetAngleAbs);
            ln.telemetry.update();
        }
        rF.setPower(0);
        rB.setPower(0);
        lF.setPower(0);
        lB.setPower(0);
    }

    public void strafe(double power) {
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if (getStatus() != Mark_5.Status.STRAFING) {
            startAngle = getHeading();
        }
        this.setStatus(Mark_5.Status.STRAFING);
        double turnOffset;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }else{
            turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }
        lF.setPower(power+turnOffset);
        lB.setPower(-power+turnOffset);
        rF.setPower(-power-turnOffset);
        rB.setPower(power-turnOffset);
    }
    /*public void strafe(double power, double distance){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if (!(robotStatus == Mark_5.Status.STRAFING)) {
            startAngle = getHeading();
        }
        this.setStatus(Mark_5.Status.STRAFING);
        double turnOffset;
        while(Math.abs(distance-(deadWheel.getCurrentPosition()*odometryWheelCirc/ticksPerOdometryWheel)) > distanceAccuracy){
            if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }else{
                turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }
            lF.setPower(power+turnOffset);
            lB.setPower(-power+turnOffset);
            rF.setPower(-power-turnOffset);
            rB.setPower(power-turnOffset);
            if(ln.isStopRequested()){return;}
        }
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }*/

    public void stopDrive(){
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }

    public void angleStrafe(double power, double angle){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if (getStatus() != Mark_5.Status.ANGLE_STRAFING) {
            startAngle = getHeading();
        }
        this.setStatus(Mark_5.Status.ANGLE_STRAFING);
        double turnOffset;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }else{
            turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }
        lF.setPower(power*Math.sin(angle + Math.PI/4)+turnOffset);
        lB.setPower(power*Math.sin(angle - Math.PI/4)+turnOffset);
        rB.setPower(power*Math.sin(angle + Math.PI/4)-turnOffset);
        rF.setPower(power*Math.sin(angle - Math.PI/4)-turnOffset);
    }
    public void goToDeltaPosition(double power, double meters, double targetAngle){
        turn(power, targetAngle);
        forward(power, meters);
    }
    public void goToAbsolutePosition(double power, double x, double y)throws InterruptedException{
        double deltaX = x-odometryX;
        double deltaY = y-odometryY;
        double angle = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaY, deltaX);
        ln.telemetry.addData("targetAngle:", angle);
        ln.telemetry.addData("Distance:", distance);
        ln.telemetry.update();
        Thread.sleep(5000);
        goToDeltaPosition(distance, angle, power);
    }
    //Misc methods
    public double getHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.getPosition();
        return ACMath.toStandardAngle(Math.toRadians(angles.firstAngle)+initialAngle);
    }
}
