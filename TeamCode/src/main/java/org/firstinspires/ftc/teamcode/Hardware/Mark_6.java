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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Odometry;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


public class Mark_6 {
    private Mark_5.Status robotStatus;
    public void setStatus(Mark_5.Status s){
        robotStatus = s;
    }
    public Mark_5.Status getStatus(){
        return robotStatus;
    }

    public DcMotor lF, lB, rF, rB, intakeL, intakeR, lift;

    BNO055IMU imu;
    Orientation angles;

    //wheel Diameter in cm
    final double wheelDiameter = 5.08;
    //wheel Circumference in meters
    final double wheelCirc = wheelDiameter * Math.PI/100;
    //ticks per full revolution of the motor axle
    final double ticksPerMotorRev = 8192;
    final double motorGearRatio = 1;
    //ticks per full revolution of the wheel
    final double ticksPer = motorGearRatio*ticksPerMotorRev;
    double LENGTH = 0.38121;
    double naturalMiddleMovementPerRadian = 0.082572;

    public static final double middleDeadWheelCorrection = 66.17647058823529;

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    public Odometry odo;
    double initialAngle;

    final double angleAccuracy = 0.01;
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

        lF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rB .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //***Odometry init***
        if(!teleOpInit) {
            this.LENGTH = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
            this.naturalMiddleMovementPerRadian = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        }
        odo = new Odometry(ln, new DeadWheel(lB), new DeadWheel(rF), new DeadWheel(lF), wheelDiameter, ticksPerMotorRev, motorGearRatio, LENGTH, naturalMiddleMovementPerRadian, startX, startY, startAngle, -1, 1, -1);
        initialAngle = startAngle;

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

    public void setOdometryPosition(double x, double y){
        odo.setX(x);
        odo.setY(y);
    }
    public void resetAngle(double angle){
        odo.resetAngle(angle);
    }

    //These functions use PD control to set the velocity of each dead wheel
    final double KP = 0.5;
    final double KD = 1;
    double lastErrorL;
    double lastTimeL;
    public void updatePDVelocityL(double v){
        double error = v-odo.getVelocityL();
        double time = System.currentTimeMillis();
        double power = Range.clip( KP*error+KD*(error-lastErrorL)/(time-lastTimeL) + v, -1, 1);
        lF.setPower(power);
        lB.setPower(power);
        ln.telemetry.addData("Left Power", power);
        lastErrorL = error;
        lastTimeL = time;
        ln.telemetry.addData("diffL",1000*(time-lastTimeL));
    }
    double lastErrorR;
    double lastTimeR;
    public void updatePDVelocityR(double v){
        double error = v-odo.getVelocityR();
        double time = System.currentTimeMillis();
        double power = Range.clip(KP*error+KD*(error-lastErrorR)/(time-lastTimeR) + v, -1, 1);
        rF.setPower(power);
        rB.setPower(power);
        ln.telemetry.addData("Right power", power);
        lastErrorR = error;
        lastTimeR = time;
        ln.telemetry.addData("diffR",1000*(time-lastTimeR));
    }

    //Movement methods
    double startAngle;
    final double maxCorrectionAngle = Math.PI/4;
    public void forward(double power){
        if(this.getStatus() != Mark_5.Status.FORWARD){
            startAngle = getHeading();
            this.setStatus(Mark_5.Status.FORWARD);
        }
        double angleError;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
        }else{
            angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = angleError/maxCorrectionAngle;
        lF.setPower(Range.clip(power+turnOffset,-1,1));
        lB.setPower(Range.clip(power+turnOffset,-1,1));
        rF.setPower(Range.clip(power-turnOffset,-1,1));
        rB.setPower(Range.clip(power-turnOffset,-1,1));
    }
    public void forward(double power, double meters, boolean stop){
        if(this.getStatus() != Mark_5.Status.FORWARD){
            startAngle = getHeading();
            this.setStatus(Mark_5.Status.FORWARD);
        }
        double angleError;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
        }else{
            angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = angleError/maxCorrectionAngle;
        double startEncoder = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
        double currentEncoder = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
        double distanceTraveled = wheelCirc*(currentEncoder-startEncoder)/ticksPer;
        double distanceError = meters-distanceTraveled;
        double lowestPower = 0.1;
        if(distanceError > 0){
            while(distanceError > 0){
                double newPower = power*(Range.clip(distanceError/meters+lowestPower, -(1-turnOffset), (1-turnOffset)));
                if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                    angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
                }else{
                    angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
                }
                turnOffset = angleError/maxCorrectionAngle;
                if(distanceError > 0) {
                    lF.setPower(newPower + turnOffset);
                    lB.setPower(newPower + turnOffset);
                    rF.setPower(newPower - turnOffset);
                    rB.setPower(newPower - turnOffset);
                }else{
                    lF.setPower(-newPower + turnOffset);
                    lB.setPower(-newPower + turnOffset);
                    rF.setPower(-newPower - turnOffset);
                    rB.setPower(-newPower - turnOffset);
                }
                if(Math.abs(angleError) > maxCorrectionAngle/2){
                    turn(0.5, startAngle);
                }
                currentEncoder = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
                distanceTraveled = wheelCirc*(currentEncoder-startEncoder)/ticksPer;
                distanceError = meters-distanceTraveled;
            }
            if(stop) {
                lF.setPower(-power);
                lB.setPower(-power);
                rF.setPower(-power);
                rB.setPower(-power);
            }
        }else{
            while(distanceError < 0){
                double newPower = power*(Range.clip(distanceError/meters+lowestPower, -(1-turnOffset), (1-turnOffset)));
                if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                    angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
                }else{
                    angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
                }
                turnOffset = angleError/maxCorrectionAngle;
                if(distanceError > 0) {
                    lF.setPower(newPower + turnOffset);
                    lB.setPower(newPower + turnOffset);
                    rF.setPower(newPower - turnOffset);
                    rB.setPower(newPower - turnOffset);
                }else{
                    lF.setPower(-newPower + turnOffset);
                    lB.setPower(-newPower + turnOffset);
                    rF.setPower(-newPower - turnOffset);
                    rB.setPower(-newPower - turnOffset);
                }
                if(Math.abs(angleError) > maxCorrectionAngle/2){
                    turn(0.5, startAngle);
                }
                currentEncoder = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
                distanceTraveled = wheelCirc*(currentEncoder-startEncoder)/ticksPer;
                distanceError = meters-distanceTraveled;
                }
            if(stop) {
                lF.setPower(power);
                lB.setPower(power);
                rF.setPower(power);
                rB.setPower(power);
            }
            }
        stopDrive();
    }

    public void turn(double power){
        this.setStatus(Mark_5.Status.TURNING);
        rF.setPower(power);
        rB.setPower(power);
        lF.setPower(-power);
        lB.setPower(-power);
    }
    public void turn(double power, double targetAngle){
        double angle = getHeading();
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
            angle = getHeading();
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
    public void turn(double power, double targetAngle, boolean override){
        double angle = getHeading();
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
            angle = getHeading();
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
        if(this.getStatus() != Mark_5.Status.STRAFING){
            startAngle = getHeading();
            this.setStatus(Mark_5.Status.STRAFING);
        }
        double error;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            error = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
        }else{
            error = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = error/maxCorrectionAngle;
        lF.setPower(Range.clip(power+turnOffset, -1, 1));
        lB.setPower(Range.clip(-power+turnOffset, -1, 1));
        rF.setPower(Range.clip(-power-turnOffset, -1, 1));
        rB.setPower(Range.clip(power-turnOffset, -1, 1));
    }
    public void strafe(double power, double meters, boolean stop){
        if(this.getStatus() != Mark_5.Status.STRAFING){
            startAngle = getHeading();
            this.setStatus(Mark_5.Status.STRAFING);
        }
        double angleError;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
        }else{
            angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = angleError/maxCorrectionAngle;
        double startEncoder = odo.middle.getCurrentPosition();
        double currentEncoder = odo.middle.getCurrentPosition();
        double distanceTraveled = middleDeadWheelCorrection*wheelCirc*(currentEncoder-startEncoder)/ticksPer;
        double distanceError = meters-distanceTraveled;
        double lowestPower = 0.1;
        if(distanceError > 0){
        while(distanceError>0){
            if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
            }else{
                angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
            }
            turnOffset = angleError/maxCorrectionAngle;
            double newPower = power*(Range.clip(distanceError/meters+lowestPower, -(1-turnOffset), (1-turnOffset)));
            if(distanceError > 0) {
                lF.setPower(newPower + turnOffset);
                lB.setPower(-newPower + turnOffset);
                rF.setPower(-newPower - turnOffset);
                rB.setPower(newPower - turnOffset);
            }else if (distanceError < 0) {
                lF.setPower(-newPower + turnOffset);
                lB.setPower(newPower + turnOffset);
                rF.setPower(newPower - turnOffset);
                rB.setPower(-newPower - turnOffset);
            }
            if(Math.abs(angleError) > maxCorrectionAngle/2){
                turn(0.5, startAngle);
            }
            currentEncoder = odo.middle.getCurrentPosition();
            distanceTraveled = 66.17647058823529*0.0508*Math.PI*(currentEncoder-startEncoder)/8192;
            distanceError = meters-distanceTraveled;
            ln.telemetry.addData("Distance error", distanceError);
            ln.telemetry.addData("angle error", angleError);
            ln.telemetry.addData("lF motor power", lF.getPower());
            ln.telemetry.update();
            if(ln.isStopRequested()){return;}
        }
        if(stop){
            lF.setPower(-power);
            lB.setPower(power);
            rF.setPower(power);
            rB.setPower(-power);
        }
        }else{
            while(distanceError<0){
                if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                    angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
                }else{
                    angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
                }
                turnOffset = angleError/maxCorrectionAngle;
                double newPower = power*(Range.clip(distanceError/meters+lowestPower, -(1-turnOffset), (1-turnOffset)));
                if(distanceError > 0) {
                    lF.setPower(newPower + turnOffset);
                    lB.setPower(-newPower + turnOffset);
                    rF.setPower(-newPower - turnOffset);
                    rB.setPower(newPower - turnOffset);
                }else if (distanceError < 0) {
                    lF.setPower(-newPower + turnOffset);
                    lB.setPower(newPower + turnOffset);
                    rF.setPower(newPower - turnOffset);
                    rB.setPower(-newPower - turnOffset);
                }
                if(Math.abs(angleError) > maxCorrectionAngle/2){
                    turn(0.5, startAngle);
                }
                currentEncoder = odo.middle.getCurrentPosition();
                distanceTraveled = 66.17647058823529*0.0508*Math.PI*(currentEncoder-startEncoder)/8192;
                distanceError = meters-distanceTraveled;
                ln.telemetry.addData("Distance error", distanceError);
                ln.telemetry.addData("angle error", angleError);
                ln.telemetry.addData("lF motor power", lF.getPower());
                ln.telemetry.update();
                if(ln.isStopRequested()){return;}
            }
            if(stop){
                lF.setPower(power);
                lB.setPower(-power);
                rF.setPower(-power);
                rB.setPower(power);
            }
        }
        stopDrive();
    }

    //archs the robot at a controllable radius. Negative radius turns right positve turns left
    public void arch(double v, double r){
        double rightLeftRatio = (r+LENGTH/2)/(r-LENGTH/2);
            if (rightLeftRatio > 1) {
                updatePDVelocityR(v);
                updatePDVelocityL(v / rightLeftRatio);
            } else if (rightLeftRatio < 1) {
                updatePDVelocityL(v);
                updatePDVelocityR(v * rightLeftRatio);
            } else {
                updatePDVelocityR(v);
                updatePDVelocityL(v);
            }
            rightLeftRatio = (r + LENGTH / 2) / (r - LENGTH / 2);
    }
    public void arch(double v, double r, double meters, boolean stop){
        double rightLeftRatio = (r+LENGTH/2)/(r-LENGTH/2);
        double startEncoder = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
        double currentEncoder = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
        double distanceTraveled = wheelCirc*(currentEncoder-startEncoder)/ticksPer;
        double error = meters-distanceTraveled;
        if(error>0) {
            while (error > 0) {
                rightLeftRatio = (r + LENGTH / 2) / (r - LENGTH / 2);
                if (error > 0) {
                    if (rightLeftRatio > 1) {
                        updatePDVelocityR(v);
                        updatePDVelocityL(v / rightLeftRatio);
                    } else if (rightLeftRatio < 1) {
                        updatePDVelocityL(v);
                        updatePDVelocityR(v * rightLeftRatio);
                    } else {
                        updatePDVelocityR(v);
                        updatePDVelocityL(v);
                    }
                } else {
                    if (rightLeftRatio > 1) {
                        updatePDVelocityR(-v);
                        updatePDVelocityL(-v / rightLeftRatio);
                    } else if (rightLeftRatio < 1) {
                        updatePDVelocityL(-v);
                        updatePDVelocityR(-v * rightLeftRatio);
                    } else {
                        updatePDVelocityR(-v);
                        updatePDVelocityL(-v);
                    }
                }
                currentEncoder = (odo.left.getCurrentPosition() + odo.right.getCurrentPosition()) / 2;
                distanceTraveled = wheelCirc * (currentEncoder - startEncoder) / ticksPer;
                error = meters - distanceTraveled;
                if(ln.isStopRequested()){return;}
            }
            if(stop) {
                lF.setPower(-v);
                lB.setPower(-v);
                rF.setPower(-v);
                rB.setPower(-v);
            }
        }else{
            while (error < 0) {
                rightLeftRatio = (r + LENGTH / 2) / (r - LENGTH / 2);
                if (error > 0) {
                    if (rightLeftRatio > 1) {
                        updatePDVelocityR(v);
                        updatePDVelocityL(v / rightLeftRatio);
                    } else if (rightLeftRatio < 1) {
                        updatePDVelocityL(v);
                        updatePDVelocityR(v * rightLeftRatio);
                    } else {
                        updatePDVelocityR(v);
                        updatePDVelocityL(v);
                    }
                } else {
                    if (rightLeftRatio > 1) {
                        updatePDVelocityR(-v);
                        updatePDVelocityL(-v / rightLeftRatio);
                    } else if (rightLeftRatio < 1) {
                        updatePDVelocityL(-v);
                        updatePDVelocityR(-v * rightLeftRatio);
                    } else {
                        updatePDVelocityR(-v);
                        updatePDVelocityL(-v);
                    }
                }
                currentEncoder = (odo.left.getCurrentPosition() + odo.right.getCurrentPosition()) / 2;
                distanceTraveled = wheelCirc * (currentEncoder - startEncoder) / ticksPer;
                error = meters - distanceTraveled;
                if(ln.isStopRequested()){return;}
            }
            if(stop) {
                lF.setPower(v);
                lB.setPower(v);
                rF.setPower(v);
                rB.setPower(v);
            }
        }
        stopDrive();
    }

    //angle strafe allows the robot to move in any direction without turning
    public void angleStrafe(double power, double angle){
        if(this.getStatus() != Mark_5.Status.ANGLE_STRAFING){
            startAngle = getHeading();
            this.setStatus(Mark_5.Status.ANGLE_STRAFING);
        }
        double angleError;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
        }else{
            angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = angleError/maxCorrectionAngle;
        lF.setPower(Range.clip(power*Math.sin(angle + Math.PI/4), -(1-turnOffset), (1-turnOffset))+turnOffset);
        lB.setPower(Range.clip(power*Math.sin(angle - Math.PI/4), -(1-turnOffset), (1-turnOffset))+turnOffset);
        rB.setPower(Range.clip(power*Math.sin(angle + Math.PI/4), -(1-turnOffset), (1-turnOffset))-turnOffset);
        rF.setPower(Range.clip(power*Math.sin(angle - Math.PI/4), -(1-turnOffset), (1-turnOffset))-turnOffset);
    }
    public void angleStrafe(double power, double angle, double meters){
        double startEncoderY = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
        double currentEncoderY = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
        double distanceTraveledY = wheelCirc*(currentEncoderY-startEncoderY)/ticksPer;
        double distanceErrorY = meters*Math.sin(angle)-distanceTraveledY;
        double startEncoderX = odo.middle.getCurrentPosition();
        double currentEncoderX = odo.middle.getCurrentPosition();
        double distanceTraveledX = middleDeadWheelCorrection*wheelCirc*(currentEncoderX-startEncoderX)/ticksPer;
        double distanceErrorX = meters*Math.cos(angle) - distanceTraveledX;
        double lowestPower = 0.1;
        while(Math.abs(Math.hypot(distanceErrorX, distanceErrorY)) > distanceAccuracy){
            angleStrafe(power, Math.atan2(distanceErrorY, distanceErrorX));
            currentEncoderX = odo.middle.getCurrentPosition();
            distanceTraveledX = middleDeadWheelCorrection*wheelCirc*(currentEncoderX-startEncoderX)/ticksPer;
            distanceErrorX = meters*Math.cos(angle) - distanceTraveledX;
            currentEncoderY = (odo.left.getCurrentPosition()+odo.right.getCurrentPosition())/2;
            distanceTraveledY = wheelCirc*(currentEncoderY-startEncoderY)/ticksPer;
            distanceErrorY = meters*Math.sin(angle)-distanceTraveledY;
        }
    }

    //Can turn and strafe at the same time to save time in auto
    //This is the only function that REQUIRES odometry tracking (all of the functions require deadwheels)
    public void turningStrafe(double power, double targetHeading, double movementAngle, double meters) {
        angleStrafe(power, movementAngle - getHeading() + Math.PI / 2);
        double strafeAngle = movementAngle - getHeading() + Math.PI / 2;
        double angle = getHeading();
        double targetAngleDelta;
        if (ACMath.compassAngleShorter(targetHeading, angle)) {
            targetAngleDelta = ACMath.toCompassAngle(targetHeading) - ACMath.toCompassAngle(angle);
        } else {
            targetAngleDelta = ACMath.toStandardAngle(targetHeading) - ACMath.toStandardAngle(angle);
        }
        double targetAngleAbs = Math.abs(targetAngleDelta);
        boolean sw = false;
        if (targetAngleDelta > 0)
            sw = true;
        if (targetAngleDelta < 0)
            sw = false;
        int decreaseRate = 1;
        double targetX = meters*Math.cos(movementAngle)+odo.getX();
        double targetY = meters*Math.sin(movementAngle)+odo.getY();
        double deltaX = targetX-odo.getX();
        double deltaY = targetY-odo.getY();
        while (targetAngleAbs >= angleAccuracy && Math.abs(Math.hypot(deltaX, deltaY)) > distanceAccuracy) {
            deltaX = targetX-odo.getX();
            deltaY = targetY-odo.getY();
            strafeAngle = Math.atan2(deltaY, deltaX) - angle + Math.PI / 2;
            //If change is negative, robot turns in negative direction
            if (targetAngleDelta < 0) {
                //Turn right
                rF.setPower(-power/(2*decreaseRate) + power*Math.sin(strafeAngle - Math.PI/4)/2);
                rB.setPower(-power/(2*decreaseRate) + power*Math.sin(strafeAngle + Math.PI/4)/2);
                lF.setPower(power/(2*decreaseRate) + power*Math.sin(strafeAngle + Math.PI/4)/2);
                lB.setPower(power/(2*decreaseRate) + power*Math.sin(strafeAngle - Math.PI/4)/2);
                if (!sw) {
                    decreaseRate++;
                }
                sw = true;
            }
            if (targetAngleDelta > 0) {
                //Turn left
                lF.setPower(-power/(2*decreaseRate) + power*Math.sin(strafeAngle + Math.PI/4)/2);
                lB.setPower(-power/(2*decreaseRate) + power*Math.sin(strafeAngle - Math.PI/4)/2);
                rF.setPower(power/(2*decreaseRate) + power*Math.sin(strafeAngle + Math.PI/4)/2);
                rB.setPower(power/(2*decreaseRate) + power*Math.sin(strafeAngle - Math.PI/4)/2);
                if (sw) {
                    decreaseRate++;
                }
                sw = false;
            }
            angle = getHeading();
            if (ACMath.compassAngleShorter(targetHeading, angle)) {
                targetAngleDelta = ACMath.toCompassAngle(targetHeading) - ACMath.toCompassAngle(angle);
            } else {
                targetAngleDelta = ACMath.toStandardAngle(targetHeading) - ACMath.toStandardAngle(angle);
            }
            targetAngleAbs = Math.abs(targetAngleDelta);
            if (ln.isStopRequested()) return;
            ln.telemetry.addData("targetAngleDelta", targetAngleDelta);
            ln.telemetry.addData("odometryAngle", angle);
            ln.telemetry.addData("targetAngle", targetHeading);
            ln.telemetry.addData("Odo X", odo.getX());
            ln.telemetry.addData("Odo Y", odo.getY());
            ln.telemetry.addData("Delta X", deltaX);
            ln.telemetry.addData("Delta Y", deltaY);
            ln.telemetry.update();
        }
    }

    public void stopDrive(){
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }

    public void goToDeltaPosition(double power, double meters, double targetAngle){
        turn(power, targetAngle);
        forward(power, meters, true);
    }
    public void goToDeltaPosition(double power, double meters, double targetAngle, boolean stop){
        turn(power, targetAngle);
        forward(power, meters, stop);
    }
    public void goToDeltaCurvePosition(double v, double x, double y){
        double angleBetween;
        double heading = getHeading();
        if(ACMath.compassAngleShorter(Math.atan2(y, x), heading)) {
            angleBetween = ACMath.toCompassAngle(Math.atan2(y, x)) - ACMath.toCompassAngle(heading);
        }else{
            angleBetween = ACMath.toStandardAngle(Math.atan2(y, x)) - ACMath.toStandardAngle(heading);
        }
        double radius;
        if(angleBetween > 0){
            radius = Math.hypot(x, y)*Math.sin(Math.PI/2-angleBetween)/Math.sin(2*angleBetween);
            if(Math.abs(angleBetween) > Math.PI/2) {
                arch(v, radius, -ACMath.chordToArch(Math.hypot(x, y), 2*angleBetween), true);
            }else{
                arch(v, radius, ACMath.chordToArch(Math.hypot(x, y), 2*angleBetween), true);
            }
        }else if(angleBetween < 0){
            radius = Math.hypot(x, y)*Math.sin(-Math.PI/2-angleBetween)/Math.sin(Math.PI-2*(Math.PI/2-angleBetween));
            if(Math.abs(angleBetween) > Math.PI/2) {
                arch(v, radius, -ACMath.chordToArch(Math.hypot(x, y), Math.PI-2*(Math.PI/2-angleBetween)), true);
            }else{
                arch(v, radius, ACMath.chordToArch(Math.hypot(x, y), Math.PI-2*(Math.PI/2-angleBetween)), true);
            }
        }else if(angleBetween == Math.PI){
            forward(v, -Math.hypot(x, y), true);
        }else{
            forward(v, Math.hypot(x, y), true);
        }
    }
    public void goToDeltaCurvePosition(double v, double x, double y, boolean stop){
        double angleBetween;
        double heading = getHeading();
        if(ACMath.compassAngleShorter(Math.atan2(y, x), heading)) {
            angleBetween = ACMath.toCompassAngle(Math.atan2(y, x)) - ACMath.toCompassAngle(heading);
        }else{
            angleBetween = ACMath.toStandardAngle(Math.atan2(y, x)) - ACMath.toStandardAngle(heading);
        }
        double radius;
        if(angleBetween > 0){
            radius = Math.hypot(x, y)*Math.sin(Math.PI/2-angleBetween)/Math.sin(2*angleBetween);
            if(Math.abs(angleBetween) > Math.PI/2) {
                arch(v, radius, -ACMath.chordToArch(Math.hypot(x, y), 2*angleBetween), stop);
            }else{
                arch(v, radius, ACMath.chordToArch(Math.hypot(x, y), 2*angleBetween), stop);
            }
        }else if(angleBetween < 0){
            radius = Math.hypot(x, y)*Math.sin(-Math.PI/2-angleBetween)/Math.sin(Math.PI-2*(Math.PI/2-angleBetween));
            if(Math.abs(angleBetween) > Math.PI/2) {
                arch(v, radius, -ACMath.chordToArch(Math.hypot(x, y), Math.PI-2*(Math.PI/2-angleBetween)), stop);
            }else{
                arch(v, radius, ACMath.chordToArch(Math.hypot(x, y), Math.PI-2*(Math.PI/2-angleBetween)), stop);
            }
        }else if(angleBetween == Math.PI){
            forward(v, -Math.hypot(x, y), true);
        }else{
            forward(v, Math.hypot(x, y), true);
        }
    }

    public void goToAbsolutePosition(double power, double x, double y){
        double deltaX = x-odo.getX();
        double deltaY = y-odo.getY();
        double angle = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaY, deltaX);
        ln.telemetry.addData("targetAngle:", angle);
        ln.telemetry.addData("Distance:", distance);
        ln.telemetry.update();
        goToDeltaPosition(distance, angle, power);
    }
    public void goToAbsolutePosition(double power, double x, double y, boolean stop){
        double deltaX = x-odo.getX();
        double deltaY = y-odo.getY();
        double angle = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaY, deltaX);
        ln.telemetry.addData("targetAngle:", angle);
        ln.telemetry.addData("Distance:", distance);
        ln.telemetry.update();
        goToDeltaPosition(distance, angle, power, stop);
    }
    public void goToAbsoluteCurvePosition(double power, double x, double y){
        double deltaX = x-odo.getX();
        double deltaY = y-odo.getY();
        goToDeltaCurvePosition(power, deltaX, deltaY);
    }
    public void goToAbsoluteCurvePosition(double power, double x, double y, boolean stop){
        double deltaX = x-odo.getX();
        double deltaY = y-odo.getY();
        goToDeltaCurvePosition(power, deltaX, deltaY, stop);
    }

    //Misc methods
    public double getHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.getPosition();
        return ACMath.toStandardAngle(Math.toRadians(angles.firstAngle)+initialAngle);
    }
}
