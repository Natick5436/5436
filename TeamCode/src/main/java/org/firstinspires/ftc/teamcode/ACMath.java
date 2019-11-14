package org.firstinspires.ftc.teamcode;

public class ACMath{
    public static double toCompassAngle(double standardAngle){
        while(standardAngle > 2*Math.PI){
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
        while(compassAngle < -Math.PI){
            compassAngle += 2*Math.PI;
        }
        return compassAngle;
    }

    static boolean compassAngleShorter(double currentAngle, double targetAngle){
        double compassDelta = toCompassAngle(targetAngle)-toCompassAngle(currentAngle);
        double standardDelta = toStandardAngle(targetAngle)-toStandardAngle(currentAngle);
        if(compassDelta < standardDelta){
            return true;
        }else{
            return false;
        }
    }
}
