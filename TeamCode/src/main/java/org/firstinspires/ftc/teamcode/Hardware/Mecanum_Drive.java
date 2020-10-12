package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.Math.AStarSearch;
import org.firstinspires.ftc.teamcode.Math.PathInterface;
import org.firstinspires.ftc.teamcode.Math.PurePursuitPath;
import org.firstinspires.ftc.teamcode.Math.Waypoint;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.AngleTracker;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.DoubleFunction;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.PositionTracker;

import java.util.ArrayList;

public class Mecanum_Drive {
    private double wheelRadius;
    private double LENGTH_Y;
    private double LENGTH_X;
    private double motorMaxRPM;
    private double rpmToRadPerSec = 0.10472;
    private double desiredVoltage = 12.0;
    private double forwardVPerStrafeV = 1.071;
    private double[][] inverseKinematicsMatrix;
    public DcMotor lF, lB, rF, rB;
    private LinearOpMode ln;

    //Sensors
    private DcMotor.RunMode runMode;
    private VoltageSensor voltage;
    private AngleTracker angle;
    private PositionTracker pos;

    private DcMotor[] wheelArray;
    public Mecanum_Drive(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, DcMotor.RunMode runMode, double wheelRadius, double LENGTH_X, double LENGTH_Y, double motorMaxRPM){
        this.lF = lF;
        this.lB = lB;
        this.rF = rF;
        this.rB = rB;
        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setMode(runMode);
        lB.setMode(runMode);
        rF.setMode(runMode);
        rB.setMode(runMode);
        this.runMode = runMode;
        this.wheelRadius = wheelRadius;
        this.LENGTH_X = LENGTH_X;
        this.LENGTH_Y = LENGTH_Y;
        this.motorMaxRPM = motorMaxRPM;
        this.wheelArray = new DcMotor[]{this.lF, this.rF, this.lB, this.rB};
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                                                        {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                                                        {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                                                        {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }

    public void attachLinearOpMode(LinearOpMode linearOpMode){
        ln = linearOpMode;
    }

    public boolean enableBrakes(){
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(runMode != DcMotor.RunMode.RUN_USING_ENCODER){
            return false;
        }else{
            return true;
        }
    }
    public void disableBrakes(){
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }




    /****Attaching and using Sensors****/

    public boolean isDriveEncoders(){
        return runMode == DcMotor.RunMode.RUN_USING_ENCODER;
    }

    //Voltage sensor
    public void attachVoltageSensor(VoltageSensor voltage){
        this.voltage = voltage;
    }
    public boolean isVoltage(){
        return voltage != null;
    }
    public VoltageSensor getVoltageSensor(){
        return voltage;
    }
    public double getVoltage(){
        return voltage.getVoltage();
    }

    //Angle Tracking
    public void attachAngleTracker(AngleTracker angle){
        this.angle = angle;
    }
    public boolean isAngleTracker(){
        return angle != null;
    }
    public AngleTracker getAngleTracker(){
        return angle;
    }
    public double getAngle(){
        return angle.getAngle();
    }

    //Position Tracking
    public void attachPositionTracker(PositionTracker pos){
        this.pos = pos;
    }
    public boolean isPosTracker(){
        return pos != null;
    }
    public PositionTracker getPositionTracker(){
        return pos;
    }
    public double getX(){
        return pos.getX();
    }
    public double getY(){
        return pos.getY();
    }




    /*****Basic Movement Methods (No position tracking or path following)*****/

    //All in one movement function
    // Parameters
    // index 0: X Velocity (m/s) aka velocity forward and backward (from robot's POV). Positive=Forward, Negative=Backwards
    // index 1: Y Velocity (m/s) aka velocity left and right (from robot's POV). Positive=Left, Negative=Right
    // index 2: Angular Velocity (rad/s) aka velocity counterclockwise and clockwise (from robot's POV). Positive=counterclockwise, Negative=clockwise
    public double[] inverseKinematics(double[] velocities){
        double[] wheelSpeeds = new double[4];
        velocities[1] = forwardVPerStrafeV*velocities[1];
        for(int i=0; i<inverseKinematicsMatrix.length; i++){
            double sum = 0;
            for(int j=0; j<inverseKinematicsMatrix[i].length; j++){
                sum+=inverseKinematicsMatrix[i][j]*velocities[j];
            }
            wheelSpeeds[i]=sum/wheelRadius;
        }
        for(int m=0; m<wheelArray.length; m++){
            wheelArray[m].setPower(wheelSpeeds[m]/(rpmToRadPerSec*motorMaxRPM));
        }
        return wheelSpeeds;
    }

    //Stops all 4 motors. Usually put after done with a movement so motors don't keep running
    public void stopDrive(){
        for(DcMotor wheel : wheelArray){
            wheel.setPower(0);
        }
    }

    //sets power to all the motors at once. Used for any purpose you can dream up.
    // Indexes of parameter
    // 0: leftFront
    // 1: rightFront
    // 2: leftBack
    // 3: rightBack
    public void setPowerAll(double[] powers){
        for(int i=0; i<wheelArray.length; i++){
            wheelArray[i].setPower(powers[i]);
        }
    }




    /*****Obstacle Avoidance, Path Following All that Jazz*****/

    private boolean[][] ob;
    private double pixelsPerMeter;
    private double fieldWidthX;
    private double fieldWidthY;
    private double maxRobotRadius;

    public void attachObstacles(boolean[][] ob, double fieldWidthX, double fieldWidthY){
        double tempPixelsPerMeter = Math.min(ob.length/fieldWidthX, ob[0].length/fieldWidthY);
        boolean[][] newOb = new boolean[(int)(fieldWidthX*tempPixelsPerMeter)][(int)(fieldWidthY*tempPixelsPerMeter)];
        for(int i=0; i<newOb.length; i++){
            for (int j=0; j<newOb[i].length; j++){
                if(i<ob.length && j<ob[i].length) {
                    newOb[i][j] = ob[i][j];
                }else{
                    newOb[i][j] = false;
                }
            }
        }
        this.ob = newOb;
        this.pixelsPerMeter = tempPixelsPerMeter;
        this.fieldWidthX = fieldWidthX;
        this.fieldWidthY = fieldWidthY;
    }

    public int toPixelSpace(double meterValue){
        return (int)(pixelsPerMeter*meterValue);
    }

    public void createBuffers(double maxRobotRadius){
        this.maxRobotRadius = maxRobotRadius;
        ob = PathInterface.addSpaceLimits(ob);
        ob = PathInterface.createObstacleBuffer(ob, toPixelSpace(maxRobotRadius));
    }

    private double m;
    private double s;
    private double positionError;
    public void setPathFollowingParameters(double maxAngleCorrectVelocity /*Max Angular velocity to correct angle*/,
            double saturationAngle /*Max displacement angle before angle correction is saturated*/,
            double positionError/*How much the odometry can be off by before it stops the path and recalculates*/){
        m = maxAngleCorrectVelocity;
        s = saturationAngle;
        this.positionError = positionError;
    }


    /*
    *  THE COOL FUNCTION
    * If you set up everything correctly then this function will stop at nothing to make sure that you get
    * to the target location, this may very well turn your robot into a fighting machine so watch out
    *
    * Anyway this bad boy uses RECURSION and it melted my mind to create this
     */
    public boolean maneuverToPosition(double goalX, double goalY, double velocity, double maintainedAngle){
        boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
        AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX),  toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);

        ArrayList<AStarSearch.Node> path;
        if(aStar.getFinalPath() != null){
            path = aStar.getFinalPath();
        }else{
            return false;
        }

        path = PathInterface.condense(path);
        path = PathInterface.findShortCuts(path, instanceObstacles);
        Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
        PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
        follow.mapTimes(velocity, this);

        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < follow.getTotalPathTime()*1000.0){
            double timeSeconds = (System.currentTimeMillis()-startTime)/1000.0;
            double angleCorrection;
            if(ACMath.compassAngleShorter(maintainedAngle+getAngle(), follow.angleByTime(timeSeconds))){
                angleCorrection = ACMath.toCompassAngle(follow.angleByTime(timeSeconds)) - ACMath.toCompassAngle(maintainedAngle+getAngle());
            }else{
                angleCorrection =  ACMath.toStandardAngle(follow.angleByTime(timeSeconds)) - ACMath.toStandardAngle(maintainedAngle+getAngle());
            }
            double[] speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds)*Math.cos(follow.angleByTime(timeSeconds)-getAngle()), follow.linearVelocityOfTime(timeSeconds)*Math.sin(follow.angleByTime(timeSeconds)-getAngle()), (follow.anglePrimeByTime(timeSeconds) + m*(-Math.pow(Math.E, -Math.pow(angleCorrection/s, 2))+1)*(Math.abs(angleCorrection)/angleCorrection))});
            if(ln.isStopRequested()){
                stopDrive();
                return false;
            }
            if(Math.hypot(follow.xOfTime(timeSeconds)-getX(), follow.yOfTime(timeSeconds)-getY()) > positionError){
                stopDrive();
                ln.telemetry.addData("Off course:", "Recalculating");
                ln.telemetry.update();
                ln.sleep(500);
                return maneuverToPosition(goalX, goalY, velocity, maintainedAngle);
            }
            pos.update();
            angle.update();
        }
        stopDrive();
        return true;
    }
    //Overloaded to be angle independent. Passes a DoubleFunction to find which angle it should be traveling at.
    public boolean maneuverToPosition(double goalX, double goalY, double velocity, DoubleFunction currentAngleTiming){
        boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
        AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX),  toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);

        ArrayList<AStarSearch.Node> path;
        if(aStar.getFinalPath() != null){
            path = aStar.getFinalPath();
        }else{
            return false;
        }

        path = PathInterface.condense(path);
        path = PathInterface.findShortCuts(path, instanceObstacles);
        Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
        //purePursuitInput = PathInterface.mergePoints(purePursuitInput, 0.3);
        PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
        follow.mapTimes(velocity, this);

        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime < follow.getTotalPathTime()*1000.0){
            double timeSeconds = (System.currentTimeMillis()-startTime)/1000.0;
            double angleCorrection;
            if(ACMath.compassAngleShorter(currentAngleTiming.doubleFunction(timeSeconds/follow.getTotalPathTime()), getAngle())){
                angleCorrection = ACMath.toCompassAngle(currentAngleTiming.doubleFunction(timeSeconds/follow.getTotalPathTime())) - ACMath.toCompassAngle(getAngle());
                return maneuverToPosition(goalX, goalY, velocity, currentAngleTiming);
            }else{
                angleCorrection =  ACMath.toStandardAngle(currentAngleTiming.doubleFunction(timeSeconds/follow.getTotalPathTime())) - ACMath.toStandardAngle(getAngle());
            }
            double[] speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds)*Math.cos(follow.angleByTime(timeSeconds)-getAngle()), follow.linearVelocityOfTime(timeSeconds)*Math.sin(follow.angleByTime(timeSeconds)-getAngle()), m*(-Math.pow(Math.E, -Math.pow(angleCorrection/(Math.sqrt(2)*s), 2))+1)*(Math.abs(angleCorrection)/angleCorrection)});
            if(ln.isStopRequested()){
                stopDrive();
                return false;
            }
            if(Math.hypot(follow.xOfTime(timeSeconds)-getX(), follow.yOfTime(timeSeconds)-getY()) > positionError){
                stopDrive();
                ln.telemetry.addData("Off course:", "Recalculating");
                ln.telemetry.update();
                ln.sleep(500);
            }
        }
        stopDrive();
        return true;
    }




    /*****Random Accessors for constants and other data. Used by other classes etc.*****/

    public double getLENGTH_Y() {
        return LENGTH_Y;
    }
    public void setLENGTH_Y(double LENGTH_Y) {
        this.LENGTH_Y = LENGTH_Y;
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }
    public double getLENGTH_X() {
        return LENGTH_X;
    }
    public void setLENGTH_X(double LENGTH_X) {
        this.LENGTH_X = LENGTH_X;
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }
    public double getWheelRadius(){
        return wheelRadius;
    }
    public double getMotorMaxAngularVelocity(){
        return motorMaxRPM*rpmToRadPerSec;
    }
}
