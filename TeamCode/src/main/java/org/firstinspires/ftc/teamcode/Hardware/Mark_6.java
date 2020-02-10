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
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Odometry;

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
    final double LENGTH = 0.38121;

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
        odo = new Odometry(ln, new DeadWheel(lB), new DeadWheel(rF), new DeadWheel(lF), 5.08, 8192, 1, 0.38121, startX, startY, startAngle, -1, 1, -1);
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
    final double percentCorrection = 0.15;
    final double maxCorrectionAngle = Math.PI/16;
    public void forward(double power){
        if(this.getStatus() != Mark_5.Status.FORWARD){
            startAngle = getHeading();
            this.setStatus(Mark_5.Status.FORWARD);
        }
        double error;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            error = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
        }else{
            error = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = error/maxCorrectionAngle;
        lF.setPower(Range.clip(power+turnOffset,-1,1));
        lB.setPower(Range.clip(power+turnOffset,-1,1));
        rF.setPower(Range.clip(power+turnOffset,-1,1));
        rB.setPower(Range.clip(power+turnOffset,-1,1));
    }
    //this function only takes inputs from the front wheels of the robot
    public void forward(double power, double meters){
        double angleError;
        double turnOffset;
        double startEncoderL = odo.left.getCurrentPosition();
        double startEncoderR = odo.right.getCurrentPosition();
        double currentEncoderL = odo.left.getCurrentPosition();
        double currentEncoderR = odo.right.getCurrentPosition();
        double distanceTraveledL = 0.0508*Math.PI*(currentEncoderL-startEncoderL)/8192;
        double distanceTraveledR = 0.0508*Math.PI*(currentEncoderR-startEncoderR)/8192;
        double lastTime = System.currentTimeMillis();
        double currentTime = System.currentTimeMillis();
        double currentErrorL = meters-distanceTraveledL;
        double currentErrorR = meters-distanceTraveledR;
        double lastErrorL = meters-distanceTraveledL;
        double lastErrorR = meters-distanceTraveledR;
        double errorDerivL = ((meters-distanceTraveledL)-lastErrorL)/(currentTime-lastTime);
        double errorDerivR = ((meters-distanceTraveledR)-lastErrorR)/(currentTime-lastTime);
        double prop = 1;
        double deriv = 2;
        double lowestPower = 0.1;
        while(Math.abs(distanceTraveledL-meters) > distanceAccuracy && Math.abs(distanceTraveledR-meters) > distanceAccuracy){
            currentTime = System.currentTimeMillis();
            currentErrorL = meters-distanceTraveledL;
            currentErrorR = meters-distanceTraveledR;
            errorDerivL = (currentErrorL-lastErrorL)/(currentTime-lastTime);
            errorDerivR = (currentErrorR-lastErrorR)/(currentTime-lastTime);
            ln.telemetry.addData("Left error derivative", errorDerivL);
            ln.telemetry.addData("right error derivative", errorDerivR);
            angleError = getHeading()-startAngle;
            turnOffset = angleError/maxCorrectionAngle;
            if((meters-distanceTraveledL)>0) {
                //Two range.clips so that turn offset can't be drowned out by high numbers (over 1 or less than -1) in the main PD feedback loop
                lF.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorL/meters + deriv*errorDerivL + lowestPower), -1, 1)
                                + turnOffset, -1, 1));
                lB.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorL/meters + deriv*errorDerivL + lowestPower), -1, 1)
                                + turnOffset, -1, 1));
            }else{
                lF.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorL/meters + deriv*errorDerivL + lowestPower), -1, 1)
                                + turnOffset, -1, 1));
                lB.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorL/meters + deriv*errorDerivL + lowestPower), -1, 1)
                                + turnOffset, -1, 1));
            }
            if(meters-distanceTraveledR>0) {
                rF.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorR/meters + deriv*errorDerivR + lowestPower), -1, 1)
                                - turnOffset, -1, 1));
                rB.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorR/meters + deriv*errorDerivR + lowestPower), -1, 1)
                                - turnOffset, -1, 1));
            }else{
                rF.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorR/meters + deriv*errorDerivR + lowestPower), -1, 1)
                                - turnOffset, -1, 1));
                rB.setPower(Range.clip(
                        Range.clip(power*(prop*currentErrorR/meters + deriv*errorDerivR + lowestPower), -1, 1)
                                - turnOffset, -1, 1));
            }
            if(Math.abs(angleError)> Math.PI/4){
                turn(power, startAngle);
            }
            currentEncoderL = odo.left.getCurrentPosition();
            currentEncoderR = odo.right.getCurrentPosition();
            distanceTraveledL = 0.0508*Math.PI*(currentEncoderL-startEncoderL)/8192;
            distanceTraveledR = 0.0508*Math.PI*(currentEncoderR-startEncoderR)/8192;
            lastTime = currentTime;
            lastErrorL = currentErrorL;
            lastErrorR = currentErrorR;
            ln.telemetry.addData("distanceL",distanceTraveledL);
            ln.telemetry.addData("distanceR",distanceTraveledR);
            ln.telemetry.addData("distanceDiffL",Math.abs(distanceTraveledL-meters));
            ln.telemetry.addData("distanceDiffR",Math.abs(distanceTraveledR-meters));
            ln.telemetry.addData("left power", lF.getPower());
            ln.telemetry.addData("right power", rF.getPower());
            ln.telemetry.update();
            if(ln.isStopRequested()){
                return;
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
    public void turn(double power, double targetAngle,boolean override){
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
    public void strafe(double power, double meters){
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
        double distanceTraveled = 66.17647058823529*0.0508*Math.PI*(currentEncoder-startEncoder)/8192;
        double distanceError = meters-distanceTraveled;
        double lowestPower = 0.1;
        while(Math.abs(distanceError)>distanceAccuracy){
            if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                angleError = ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle);
            }else{
                angleError = ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle);
            }
            turnOffset = angleError/maxCorrectionAngle;
            if(distanceError > 0) {
                lF.setPower(Range.clip(power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))+distanceError/meters*turnOffset);
                lB.setPower(Range.clip(-power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))+distanceError/meters*turnOffset);
                rF.setPower(Range.clip(-power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))-distanceError/meters*turnOffset);
                rB.setPower(Range.clip(power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))-distanceError/meters*turnOffset);
            }else if (distanceError < 0) {
                lF.setPower(Range.clip(-power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))+distanceError/meters*turnOffset);
                lB.setPower(Range.clip(power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))+distanceError/meters*turnOffset);
                rF.setPower(Range.clip(power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))-distanceError/meters*turnOffset);
                rB.setPower(Range.clip(-power*Math.abs(distanceError/meters+lowestPower), -(1-distanceError/meters*turnOffset), (1-distanceError/meters*turnOffset))-distanceError/meters*turnOffset);
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
        turn(power, startAngle);
        stopDrive();
    }

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
        double deltaX = x-odo.getX();
        double deltaY = y-odo.getY();
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
