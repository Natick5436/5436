package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Mecanum_Drive {
    private double wheelRadius;
    private double LENGTH_Y;
    private double LENGTH_X;
    private double motorMaxRPM;
    private double rpmToRadPerSec = 0.10472;
    public double[][] inverseKinematicsMatrix;
    public DcMotor lF, lB, rF, rB;
    private DcMotor[] wheelArray;
    public Mecanum_Drive(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double wheelRadius, double LENGTH_X, double LENGTH_Y, double motorMaxRPM){
        this.lF = lF;
        this.lB = lB;
        this.rF = rF;
        this.rB = rB;
        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.wheelRadius = wheelRadius;
        this.LENGTH_X = LENGTH_X;
        this.LENGTH_X = LENGTH_Y;
        this.motorMaxRPM = motorMaxRPM;
        this.wheelArray = new DcMotor[]{this.lF, this.rF, this.lB, this.rB};
        this.inverseKinematicsMatrix = new double[][]{{1, -1, -(LENGTH_X + LENGTH_Y) / 2},
                                                        {1, 1, (LENGTH_X + LENGTH_Y) / 2},
                                                        {1, 1, -(LENGTH_X + LENGTH_Y) / 2},
                                                        {1, -1, (LENGTH_X + LENGTH_Y) / 2}};
    }

    public double[] inverseKinematics(double[] velocities){
        double[] wheelSpeeds = new double[4];
        for(int i=0; i<inverseKinematicsMatrix.length; i++){
            double sum = 0;
            for(int j=0; j<inverseKinematicsMatrix[i].length; j++){
                sum+=inverseKinematicsMatrix[i][j]*velocities[j];
            }
            wheelSpeeds[i]=4*sum/wheelRadius;
        }
        for(int m=0; m<wheelArray.length; m++){
            wheelArray[m].setPower(wheelSpeeds[m]/(rpmToRadPerSec*motorMaxRPM));
        }
        return wheelSpeeds;
    }
    public void stopDrive(){
        for(DcMotor wheel : wheelArray){
            wheel.setPower(0);
        }
    }
    public void setPowerAll(double[] powers){
        for(int i=0; i<wheelArray.length; i++){
            wheelArray[i].setPower(powers[i]);
        }
    }
}
