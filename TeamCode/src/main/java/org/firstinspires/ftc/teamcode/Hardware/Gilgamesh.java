package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Gilgamesh {
    //wheelDiameter in centimeters.
    final double wheelDiameter = 7.62;
    //wheel Circumference in meters for ease of odometry use.
    final double wheelCircum = wheelDiameter * Math.PI/100;
    //ticks per full revolution of the motor axle
    final double ticksPerMotorRev = 1120;
    final double motorGearRatio = 0.66667;
    //ticks per full revolution of the wheel
    final double ticksPer = motorGearRatio*ticksPerMotorRev;
    //distance in meters between the two encoded wheels
    final double LENGTH = 0.281260268994277;

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor left, right;
    public ModernRoboticsI2cGyro sensorGyro;
    public GyroSensor gyro;

    public double odometryX;
    public double odometryY;
    public double odometryAngle;
    double lastEncoderR, lastEncoderL;
    double currentEncoderL, currentEncoderR;

    final double angleAccuracy = 0.1;
    final double distanceAccuracy = 0.01;

    LinearOpMode ln;
    public Gilgamesh(LinearOpMode linear){
        ln = linear;
    }

    public void initialize(HardwareMap hardwareMap, double initialX, double initialY, double initialAngle){
        odometryX = initialX;
        odometryY = initialY;
        odometryAngle = initialAngle;

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        gyro = hardwareMap.gyroSensor.get("gyro");
        sensorGyro = (ModernRoboticsI2cGyro) gyro;

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastEncoderL = -left.getCurrentPosition();
        lastEncoderR = -right.getCurrentPosition();

        sensorGyro.calibrate();
        while(sensorGyro.isCalibrating()){
            if(ln.isStopRequested()){
                return;
            }
        }
        ln.telemetry.addData("Ready", "Ready");
        ln.telemetry.update();
    }
    public void goToPointLine(double targetX, double targetY){
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double deltaTargetX = targetX-odometryX;
        double deltaTargetY = targetY-odometryY;
        double targetAngleLine = Math.atan2(deltaTargetY, deltaTargetX);
        double localOdometryAngle;
        if(odometryAngle < 0){
            localOdometryAngle = (odometryAngle % 2*Math.PI) + Math.PI;
        }else if(odometryAngle > 0){
            localOdometryAngle = (odometryAngle % 2*Math.PI) - Math.PI;
        }else{
            localOdometryAngle = 0;
        }
        //Turn until currentAngle lines up with target angle
        while(Math.abs(localOdometryAngle - targetAngleLine) > angleAccuracy){
            //if angle is clockwise turn right
            if(localOdometryAngle - targetAngleLine < 0){
                left.setPower(0.9);
                right.setPower(-0.9);
            }
            //if angle is counter clockwise turn left
            else if(localOdometryAngle - targetAngleLine > 0){
                left.setPower(-0.9);
                right.setPower(0.9);
            }
            updateOdometryData();
            localOdometryAngle = odometryAngle % 2*Math.PI;
            if (ln.isStopRequested()){
                return;
            }
        }
        left.setPower(0);
        right.setPower(0);
        ln.sleep(500);
        while( (Math.sqrt(Math.pow(deltaTargetX, 2) + Math.pow(deltaTargetY, 2)) > distanceAccuracy) ){
            left.setPower(-1);
            right.setPower(-1);
            updateOdometryData();
            deltaTargetY = targetY-odometryY;
            deltaTargetX = targetX-odometryX;
            if (ln.isStopRequested()){
                return;
            }
        }
        left.setPower(0);
        right.setPower(0);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.UNKNOWN);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.UNKNOWN);
    }

    public void goToPointArch(double targetX, double targetY){
        double deltaTargetX = targetX-odometryX;
        double deltaTargetY = targetY-odometryY;

        double distance = Math.sqrt(Math.pow(deltaTargetX, 2)+Math.pow(deltaTargetY, 2));
        double rightLeftRatio = (2*LENGTH*Math.sin(2*(Math.atan2(deltaTargetY, deltaTargetX)-odometryAngle))) /
                (2*distance*Math.cos(Math.atan2(deltaTargetY, deltaTargetX)-odometryAngle) -
                        LENGTH*Math.sin(2*(Math.atan2(deltaTargetY, deltaTargetX)-odometryAngle)));
        while ( (Math.sqrt(Math.pow(deltaTargetX, 2) + Math.pow(deltaTargetY, 2)) > distanceAccuracy) ){
            if(rightLeftRatio > 1){
                right.setPower(1);
                left.setPower(1 / rightLeftRatio);
            }else if(rightLeftRatio == 1){
                right.setPower(1);
                left.setPower(1);
            }else if(rightLeftRatio < 1 && rightLeftRatio > 0){
                left.setPower(1);
                right.setPower(rightLeftRatio);
            }
            updateOdometryData();
            deltaTargetX = targetX-odometryX;
            deltaTargetY = targetY-odometryY;
            if(ln.isStopRequested()){
                break;
            }
        }
        left.setPower(0);
        right.setPower(0);
    }

    public void goToAngleControlled(double targetX, double targetY, double targetAngle){
        double deltaTargetX = targetX-odometryX;
        double deltaTargetY = targetY-odometryY;
        double deltaTargetAngle = targetAngle - odometryAngle;

        //turn to a valid angle before making arch turn
        while((odometryAngle - 2*Math.atan2(deltaTargetY, deltaTargetX) + targetAngle) > angleAccuracy){
            if(odometryAngle > 2*Math.atan2(deltaTargetY, deltaTargetX) - targetAngle){
                //turning right to reach an valid angle that is smaller than its current angle
                left.setPower(1);
                right.setPower(-1);
            }else if(odometryAngle < 2*Math.atan2(deltaTargetY, deltaTargetX) - targetAngle){
                //turning left to reach an valid angle that is larger than its current angle
                left.setPower(-1);
                right.setPower(1);
            }
            updateOdometryData();
            if(ln.isStopRequested()){
                return;
            }
        }

        deltaTargetAngle = targetAngle - odometryAngle;
        double distance = (Math.sqrt(Math.pow(deltaTargetX, 2) + Math.pow(deltaTargetY, 2)));
        double rightLeftRatio = 1.0 +
                (2*LENGTH*Math.sin(deltaTargetAngle)) /
                        (2*Math.cos(deltaTargetAngle/2)*distance - LENGTH*Math.sin(deltaTargetAngle));
        //sets the motor powers in a ratio to follow an arch to reach its target position. Updates ratio as odometry updates
        while(   (Math.sqrt(Math.pow(deltaTargetX, 2) + Math.pow(deltaTargetY, 2)) > distanceAccuracy)  ){
            if(rightLeftRatio > 1){
                right.setPower(1);
                left.setPower(1 / rightLeftRatio);
            }else if(rightLeftRatio == 1){
                right.setPower(1);
                left.setPower(1);
            }else if(rightLeftRatio < 1 && rightLeftRatio > 0){
                left.setPower(1);
                right.setPower(rightLeftRatio);
            }
            updateOdometryData();
            deltaTargetX = targetX-odometryX;
            deltaTargetY = targetY-odometryY;
            deltaTargetAngle = targetAngle - odometryAngle;
            if(ln.isStopRequested()){
                break;
            }
        }
        left.setPower(0);
        right.setPower(0);
    }

    //place this in at the end of every loop to update the values.
    public void updateOdometryData(){
        currentEncoderL = -left.getCurrentPosition();
        currentEncoderR = -right.getCurrentPosition();
        double deltaTickL = currentEncoderL - lastEncoderL;
        double deltaTickR = currentEncoderR - lastEncoderR;
        double dL = wheelCircum * deltaTickL / ticksPer;
        double dR = wheelCircum * deltaTickR / ticksPer;
        double dM = (dL + dR) / 2;

        double dAngle = (dR - dL) / LENGTH;

        if(dAngle != 0) {
            odometryX = odometryX + (dM * Math.sin(dAngle) * Math.cos(odometryAngle + (dAngle / 2)) /
                    (dAngle * Math.cos(dAngle / 2)));
            odometryY = odometryY + (dM * Math.sin(dAngle) * Math.sin(odometryAngle + (dAngle / 2)) /
                    (dAngle * Math.cos(dAngle / 2)));
            odometryAngle = odometryAngle + dAngle;
        }else{
            odometryX = odometryX + (dM * Math.cos(odometryAngle + (dAngle / 2)));
            odometryY = odometryY + (dM * Math.sin(odometryAngle + (dAngle / 2)));
            odometryAngle = odometryAngle + dAngle;
        }

        lastEncoderL = currentEncoderL;
        lastEncoderR = currentEncoderR;
    }
}
