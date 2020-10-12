package org.firstinspires.ftc.teamcode.Math;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;

public class PurePursuitPath {

    private Waypoint[] baseWaypoints;
    public Waypoint[] pathWaypoints;
    private double lookAheadDistance;
    private double pathTime;
    //has to be less that one
    private static final double overLookAheadCorrection = 0.99;

    //Constructors
    public PurePursuitPath(){
        this.baseWaypoints = new Waypoint[9];
        for(int i=0; i<baseWaypoints.length; i++){
            baseWaypoints[i]= new Waypoint(Math.cos(i*Math.PI/4), Math.sin(i*Math.PI/4), Waypoint.Type.BASE);
        }
        double lookAheadDistance = averageDistanceBetweenWaypoints(this.baseWaypoints)/4;
        if(lookAheadDistance >= smallestDistance()/2){
            this.lookAheadDistance=overLookAheadCorrection*smallestDistance()/2;
        }else{
            this.lookAheadDistance = lookAheadDistance;
        }
        calculatePath();
        pathTime = -1;
    }
    public PurePursuitPath(Waypoint[] baseWaypoints){
        this.baseWaypoints = baseWaypoints;
        double lookAheadDistance = averageDistanceBetweenWaypoints(this.baseWaypoints)/4;
        if(lookAheadDistance >= smallestDistance()/2){
            this.lookAheadDistance=overLookAheadCorrection*smallestDistance()/2;
        }else{
            this.lookAheadDistance = lookAheadDistance;
        }
        calculatePath();
        pathTime = -1;
    }
    public double averageDistanceBetweenWaypoints(Waypoint[] waypoints){
        double sum = 0;
        for(int i=1; i<waypoints.length; i++){
            sum += waypoints[i].distanceFrom(waypoints[i-1]);
        }
        return sum/(waypoints.length-1);
    }

    public PurePursuitPath(Waypoint[] baseWaypoints, double lookAheadDistance){
        this.baseWaypoints = baseWaypoints;
        if(lookAheadDistance >= smallestDistance()/2){
            this.lookAheadDistance=overLookAheadCorrection*smallestDistance()/2;
        }else{
            this.lookAheadDistance=lookAheadDistance;
        }
        calculatePath();
        pathTime = -1;
    }
    public double smallestDistance(){
        double minimum = baseWaypoints[0].distanceFrom(baseWaypoints[1]);
        for(int i=1; i<baseWaypoints.length-1; i++){
            minimum = Math.min(minimum, baseWaypoints[i].distanceFrom(baseWaypoints[i+1]));
        }
        return minimum;
    }
    public void calculatePath(){
        pathWaypoints = new Waypoint[2*baseWaypoints.length-2];
        pathWaypoints[0] = new Waypoint(baseWaypoints[0], Waypoint.Type.CURVE_END);
        pathWaypoints[pathWaypoints.length-1] = new Waypoint(baseWaypoints[baseWaypoints.length-1], Waypoint.Type.CURVE_START);
        for(int i=1; i<=baseWaypoints.length-2; i++){
            //calculates points that are lookAheadDistance away from baseWaypoint[i] while on the straight line to the next and previous points
            //calculates curve point start, on line from previous point to point i
            pathWaypoints[2*i-1] = new Waypoint(baseWaypoints[i].getX()+lookAheadDistance*Math.cos(baseWaypoints[i].angleTo(baseWaypoints[i-1])), baseWaypoints[i].getY()+lookAheadDistance*Math.sin(baseWaypoints[i].angleTo(baseWaypoints[i-1])), Waypoint.Type.CURVE_START);
            //calculates curve point end, on line from point i to next point
            pathWaypoints[2*i] = new Waypoint(baseWaypoints[i].getX()+lookAheadDistance*Math.cos(baseWaypoints[i].angleTo(baseWaypoints[i+1])), baseWaypoints[i].getY()+lookAheadDistance*Math.sin(baseWaypoints[i].angleTo(baseWaypoints[i+1])), Waypoint.Type.CURVE_END);
        }
    }

    //Initialization functions
    public void mapTimes(double velocity){
        double sumTime = 0;
        pathWaypoints[0].setTime(0);
        for(int i=0; i<pathWaypoints.length-1; i++){
            double timeToNext = universalDistanceToNext(i)/velocity;
            sumTime += timeToNext;
            pathWaypoints[i+1].setTime(sumTime);
        }
        pathTime = sumTime;
    }
    public void mapTimes(double velocity, Mecanum_Drive robot){
        double sumTime = 0;
        pathWaypoints[0].setTime(0);
        for(int i=0; i<pathWaypoints.length-1; i++){
            double timeToNext;
            if(pathWaypoints[i].getType() != Waypoint.Type.CURVE_START){
                if(Math.abs(velocity)<robot.getMotorMaxAngularVelocity()*robot.getWheelRadius()) {
                    timeToNext = universalDistanceToNext(i) / velocity;
                }else{
                    timeToNext = universalDistanceToNext(i) / (robot.getMotorMaxAngularVelocity()*robot.getWheelRadius());
                }
            }else{
                double angleTended = (pathWaypoints[i+1].angleTo(pathWaypoints[i+2])-pathWaypoints[i].angleFrom(pathWaypoints[i-1]));
                double angularVelocity = angleTended/(universalDistanceToNext(i)/velocity);
                if(robot.getMotorMaxAngularVelocity() >= (Math.abs(velocity)+Math.abs(0.5*(robot.getLENGTH_X()+robot.getLENGTH_Y())*angularVelocity))/robot.getWheelRadius()){
                    timeToNext = universalDistanceToNext(i)/velocity;
                }else{
                    double tempVelocity = robot.getMotorMaxAngularVelocity()*robot.getWheelRadius()/(1+Math.abs(0.5*(robot.getLENGTH_X()+robot.getLENGTH_Y())*angularVelocity)/velocity);
                    timeToNext = universalDistanceToNext(i)/tempVelocity;
                }
            }
            sumTime += timeToNext;
            pathWaypoints[i+1].setTime(sumTime);
        }
        pathTime = sumTime;
    }

    //Access Functions
    public double xOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() <= time && i<pathWaypoints.length-1){
            i++;
        }
        i--;
        if(i>=pathWaypoints.length-1) {
            return pathWaypoints[pathWaypoints.length - 1].getX();
        }else if(pathWaypoints[i].getType() == Waypoint.Type.CURVE_START){
            double thetaInitial = pathWaypoints[i].angleFrom(pathWaypoints[i-1]);
            double thetaFinal = pathWaypoints[i+1].angleTo(pathWaypoints[i+2]);
            double change;
            if(ACMath.compassAngleShorter(thetaInitial, thetaFinal)){
                change = ACMath.toCompassAngle(thetaFinal)-ACMath.toCompassAngle(thetaInitial);
            }else{
                change = ACMath.toStandardAngle(thetaFinal) - ACMath.toStandardAngle(thetaInitial);
            }
            double deltaTheta = change*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if (change == 0.0) {
                return pathWaypoints[i].getX() + Math.cos(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            } else if (change > 0) {
                double radius = lookAheadDistance / Math.tan(change/2);
                return pathWaypoints[i].getX() + radius * Math.cos(thetaInitial + Math.PI / 2) - radius * Math.cos(thetaInitial + deltaTheta + Math.PI / 2);
            } else {
                double radius = lookAheadDistance / Math.tan(change/2);
                return pathWaypoints[i].getX() - radius * Math.cos(thetaInitial - Math.PI / 2) + radius * Math.cos(thetaInitial + deltaTheta - Math.PI / 2);
            }
        }else{
            return pathWaypoints[i].getX() + Math.cos(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double xPrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() <= time && i<pathWaypoints.length-1){
            i++;
        }
        i--;
        if(i>=pathWaypoints.length-1){
            return Math.cos(pathWaypoints[pathWaypoints.length-2].angleTo(pathWaypoints[pathWaypoints.length-1]))*universalDistanceToNext(pathWaypoints.length-2)/(pathWaypoints[pathWaypoints.length-1].getTime()-pathWaypoints[pathWaypoints.length-1].getTime());
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            double thetaInitial = pathWaypoints[i].angleFrom(pathWaypoints[i-1]);
            double thetaFinal = pathWaypoints[i+1].angleTo(pathWaypoints[i+2]);
            double change;
            if(ACMath.compassAngleShorter(thetaInitial, thetaFinal)){
                change = ACMath.toCompassAngle(thetaFinal)-ACMath.toCompassAngle(thetaInitial);
            }else{
                change = ACMath.toStandardAngle(thetaFinal) - ACMath.toStandardAngle(thetaInitial);
            }
            double deltaTheta = change*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if(change == 0.0){
                return Math.cos(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            }else if(change>0){
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return radius*(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()))*Math.sin(thetaInitial+deltaTheta+Math.PI/2);
            }else{
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return -radius*(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()))*Math.sin(thetaInitial+deltaTheta-Math.PI/2);
            }
        }else{
            return Math.cos(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double xDoublePrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() <= time && i<pathWaypoints.length-1){
            i++;
        }
        i--;
        if(i>=pathWaypoints.length-1){
            return 0;
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            double thetaInitial = pathWaypoints[i].angleFrom(pathWaypoints[i-1]);
            double thetaFinal = pathWaypoints[i+1].angleTo(pathWaypoints[i+2]);
            double change;
            if(ACMath.compassAngleShorter(thetaInitial, thetaFinal)){
                change = ACMath.toCompassAngle(thetaFinal)-ACMath.toCompassAngle(thetaInitial);
            }else{
                change = ACMath.toStandardAngle(thetaFinal) - ACMath.toStandardAngle(thetaInitial);
            }
            double deltaTheta = change*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if(change == 0.0){
                return 0;
            }else if(change>0){
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return radius*Math.pow(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()), 2)*Math.cos(thetaInitial+deltaTheta+Math.PI/2);
            }else{
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return -radius*Math.pow(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()), 2)*Math.cos(thetaInitial+deltaTheta-Math.PI/2);
            }
        }else{
            return 0;
        }
    }
    public double yOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() <= time && i<pathWaypoints.length-1){
            i++;
        }
        i--;
        if(i>=pathWaypoints.length-1) {
            return pathWaypoints[pathWaypoints.length - 1].getY();
        }else if(pathWaypoints[i].getType() == Waypoint.Type.CURVE_START){
            double thetaInitial = pathWaypoints[i].angleFrom(pathWaypoints[i-1]);
            double thetaFinal = pathWaypoints[i+1].angleTo(pathWaypoints[i+2]);
            double change;
            if(ACMath.compassAngleShorter(thetaInitial, thetaFinal)){
                change = ACMath.toCompassAngle(thetaFinal)-ACMath.toCompassAngle(thetaInitial);
            }else{
                change = ACMath.toStandardAngle(thetaFinal) - ACMath.toStandardAngle(thetaInitial);
            }
            double deltaTheta = change*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());

            if(change == 0.0){
                return pathWaypoints[i].getY() + Math.sin(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            }else if(change>0){
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return pathWaypoints[i].getY() + radius*Math.sin(thetaInitial+Math.PI/2) - radius*Math.sin(thetaInitial+deltaTheta+Math.PI/2);
            }else{
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return pathWaypoints[i].getY() - radius*Math.sin(thetaInitial-Math.PI/2) + radius*Math.sin(thetaInitial+deltaTheta-Math.PI/2);
            }
        }else{
            return pathWaypoints[i].getY() + Math.sin(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double yPrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() <= time && i<pathWaypoints.length-1){
            i++;
        }
        i--;
        if(i>=pathWaypoints.length-1){
            return Math.sin(pathWaypoints[pathWaypoints.length-2].angleTo(pathWaypoints[pathWaypoints.length-1]))*universalDistanceToNext(pathWaypoints.length-2)/(pathWaypoints[pathWaypoints.length-1].getTime()-pathWaypoints[pathWaypoints.length-1].getTime());
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            double thetaInitial = pathWaypoints[i].angleFrom(pathWaypoints[i-1]);
            double thetaFinal = pathWaypoints[i+1].angleTo(pathWaypoints[i+2]);
            double change;
            if(ACMath.compassAngleShorter(thetaInitial, thetaFinal)){
                change = ACMath.toCompassAngle(thetaFinal)-ACMath.toCompassAngle(thetaInitial);
            }else{
                change = ACMath.toStandardAngle(thetaFinal) - ACMath.toStandardAngle(thetaInitial);
            }
            double deltaTheta = change*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if(change == 0.0){
                return Math.sin(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            }else if(change>0){
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return -radius*(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()))*Math.cos(thetaInitial+deltaTheta+Math.PI/2);
            }else{
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return radius*(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()))*Math.cos(thetaInitial+deltaTheta-Math.PI/2);
            }
        }else{
            return Math.sin(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double yDoublePrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() <= time && i<pathWaypoints.length-1){
            i++;
        }
        i--;
        if(i>=pathWaypoints.length-1){
            return 0;
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            double thetaInitial = pathWaypoints[i].angleFrom(pathWaypoints[i-1]);
            double thetaFinal = pathWaypoints[i+1].angleTo(pathWaypoints[i+2]);
            double change;
            if(ACMath.compassAngleShorter(thetaInitial, thetaFinal)){
                change = ACMath.toCompassAngle(thetaFinal)-ACMath.toCompassAngle(thetaInitial);
            }else{
                change = ACMath.toStandardAngle(thetaFinal) - ACMath.toStandardAngle(thetaInitial);
            }
            double deltaTheta = change*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if(change == 0.0){
                return 0;
            }else if(change>0){
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return radius*Math.pow(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()), 2)*Math.sin(thetaInitial+deltaTheta+Math.PI/2);
            }else{
                double radius = lookAheadDistance/Math.tan((thetaFinal-thetaInitial)/2);
                return -radius*Math.pow(change/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime()), 2)*Math.sin(thetaInitial+deltaTheta-Math.PI/2);
            }
        }else{
            return 0;
        }
    }
    //Robot controls
    public double linearVelocityOfTime(double time){
        return Math.hypot(yPrimeOfTime(time), xPrimeOfTime(time));
    }
    public double angleByTime(double time){
        return Math.atan2(yPrimeOfTime(time), xPrimeOfTime(time));
    }
    public double anglePrimeByTime(double time){
        return (yDoublePrimeOfTime(time)*xPrimeOfTime(time) - yPrimeOfTime(time)*xDoublePrimeOfTime(time)) / (Math.pow(yPrimeOfTime(time), 2) + Math.pow(xPrimeOfTime(time), 2));
    }

    //General Useful Functions
    public double universalDistanceToNext(int index){
        if(index+1 >= pathWaypoints.length){
            return 0;
        }
        if(pathWaypoints[index].getType() == Waypoint.Type.CURVE_START && !(index<1)){
            double theta;
            if(ACMath.compassAngleShorter(pathWaypoints[index+1].angleTo(pathWaypoints[index+2]), pathWaypoints[index].angleFrom(pathWaypoints[index-1]))) {
                theta = ACMath.toCompassAngle(pathWaypoints[index + 1].angleTo(pathWaypoints[index + 2])) - ACMath.toCompassAngle(pathWaypoints[index].angleFrom(pathWaypoints[index - 1]));
            }else{
                theta = ACMath.toStandardAngle(pathWaypoints[index + 1].angleTo(pathWaypoints[index + 2])) - ACMath.toStandardAngle(pathWaypoints[index].angleFrom(pathWaypoints[index - 1]));
            }
            if(theta == 0){
                return pathWaypoints[index].distanceFrom(pathWaypoints[index+1]);
            }else {
                return Math.abs(theta * lookAheadDistance / Math.tan(theta / 2));
            }
        }else{
            return pathWaypoints[index].distanceFrom(pathWaypoints[index+1]);
        }
    }
    public double getTotalPathTime(){
        return pathTime;
    }
}