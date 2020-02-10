package org.firstinspires.ftc.teamcode.Math;

public class ACMath{
    public static double toCompassAngle(double standardAngle){
        while(standardAngle >= 2*Math.PI){
            standardAngle -= 2*Math.PI;
        }
        while(standardAngle < 0){
            standardAngle += 2*Math.PI;
        }
        return standardAngle;
    }
    public static double toStandardAngle(double compassAngle){
        while(compassAngle > Math.PI){
            compassAngle -= 2*Math.PI;
        }
        while(compassAngle <= -Math.PI){
            compassAngle += 2*Math.PI;
        }
        return compassAngle;
    }

    public static boolean compassAngleShorter(double currentAngle, double targetAngle){
        double compassDelta = Math.abs(toCompassAngle(targetAngle)-toCompassAngle(currentAngle));
        double standardDelta = Math.abs(toStandardAngle(targetAngle)-toStandardAngle(currentAngle));
        if(compassDelta < standardDelta){
            return true;
        }else{
            return false;
        }
    }

    public static double archToChord(double archLength, double angleTended){
        return (archLength*Math.sin(angleTended)/(angleTended*Math.cos(angleTended/2)));
    }
    public static double chordToArch(double chordLength, double angleTended){
        return (chordLength*Math.cos(angleTended/2)*angleTended/Math.sin(angleTended));
    }
}
