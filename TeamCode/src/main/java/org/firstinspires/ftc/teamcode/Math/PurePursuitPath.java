package org.firstinspires.ftc.teamcode.Math;

public class PurePursuitPath {

    private Waypoint[] baseWaypoints;
    private Waypoint[] pathWaypoints;
    private double lookAheadDistance;
    private double pathTime;

    //Constructors
    public PurePursuitPath(){
        this.baseWaypoints = new Waypoint[9];
        for(int i=0; i<baseWaypoints.length; i++){
            baseWaypoints[i]= new Waypoint(Math.cos(i*Math.PI/4), Math.sin(i*Math.PI/4), Waypoint.Type.BASE);
        }
        this.lookAheadDistance = averageDistanceBetweenWaypoints(this.baseWaypoints)/4;
        calculatePath();
        pathTime = -1;
    }
    public PurePursuitPath(Waypoint[] baseWaypoints){
        this.baseWaypoints = baseWaypoints;
        this.lookAheadDistance = averageDistanceBetweenWaypoints(this.baseWaypoints)/4;
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
        this.lookAheadDistance=lookAheadDistance;
        calculatePath();
        pathTime = -1;
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
            double timeToNext = universalDistanceToNext(0)/velocity;
            sumTime += timeToNext;
            pathWaypoints[i+1].setTime(sumTime);
        }
        pathTime = sumTime;
    }

    //Access Functions
    public double xOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() > time && i<pathWaypoints.length-1){
            i++;
        }
        if(i>=pathWaypoints.length-1) {
            return pathWaypoints[pathWaypoints.length - 1].getX();
        }else if(pathWaypoints[i].getType() == Waypoint.Type.CURVE_START){
            //phi is the difference in the angle between the two straight lines before and after the curve
            double phi = pathWaypoints[i].angleTo(pathWaypoints[i-1])-pathWaypoints[i].angleTo(pathWaypoints[i+1]);
            double radius = lookAheadDistance*Math.tan(phi/2);
            //theta is the angle tended by the robot in the time specified
            double theta = (Math.PI-phi)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if(theta%Math.PI != 0) {
                return pathWaypoints[i].getX() + (radius * Math.sin(theta) / Math.cos(theta / 2)) * Math.cos((pathWaypoints[i].angleFrom(pathWaypoints[i - 1]) + theta) / 2);
            }else{
                return pathWaypoints[i].getX() + (radius*2) * Math.cos((pathWaypoints[i].angleFrom(pathWaypoints[i - 1]) + theta) / 2);
            }
        }else{
            return pathWaypoints[i].getX() + Math.cos(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double xPrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() > time && i<pathWaypoints.length-1){
            i++;
        }
        if(i>=pathWaypoints.length-1){
            return Math.cos(pathWaypoints[pathWaypoints.length-2].angleTo(pathWaypoints[pathWaypoints.length-1]))*universalDistanceToNext(pathWaypoints.length-2)/(pathWaypoints[pathWaypoints.length-1].getTime()-pathWaypoints[pathWaypoints.length-1].getTime());
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            //phi is the difference in the angle between the two straight lines before and after the curve
            double phi = pathWaypoints[i].angleTo(pathWaypoints[i-1])-pathWaypoints[i].angleTo(pathWaypoints[i+1]);
            double radius = lookAheadDistance*Math.tan(phi/2);
            double C = (Math.PI-phi)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            //theta is the angle tended by the robot in the time specified
            double theta = C*(time-pathWaypoints[i].getTime());
            double initialHeading = pathWaypoints[i].angleFrom(pathWaypoints[i - 1]);
            double common = Math.sin(theta)/Math.cos(theta/2);
            return C*radius*((Math.cos(theta)/Math.cos(theta/2)+Math.tan(theta/2)*common/2)*Math.cos((initialHeading+theta)/2)-
                                    Math.sin((initialHeading+theta)/2)*common/2);
        }else{
            return Math.cos(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double xDoublePrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() > time && i<pathWaypoints.length-1){
            i++;
        }
        if(i>=pathWaypoints.length-1){
            return 0;
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            //phi is the difference in the angle between the two straight lines before and after the curve
            double phi = pathWaypoints[i].angleTo(pathWaypoints[i-1])-pathWaypoints[i].angleTo(pathWaypoints[i+1]);
            double radius = lookAheadDistance*Math.tan(phi/2);
            double C = (Math.PI-phi)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            //theta is the angle tended by the robot in the time specified
            double theta = C*(time-pathWaypoints[i].getTime());
            double initialHeading = pathWaypoints[i].angleFrom(pathWaypoints[i - 1]);
            double common = Math.sin(theta)/Math.cos(theta/2);
            double sinm = Math.sin(theta);
            double sinm2 = Math.sin(theta/2);
            double cosm = Math.cos(theta);
            double cosm2 = Math.cos(theta/2);
            return radius*C*C*(((-sinm/cosm2) + sinm2*cosm/(2*cosm2*cosm2) + (sinm+sinm2*(2*cosm*cosm2+sinm2*sinm))/(4*cosm2*cosm2*cosm2))*Math.cos((initialHeading+theta)/2))
                                - Math.sin((initialHeading+theta)/2)*(cosm/cosm2 + sinm2*sinm/(cosm2*cosm2))/2
                                - (1.0/2.0)*(Math.sin((initialHeading+theta)/2)*(cosm*cosm2+sinm*sinm2)/(cosm2*cosm2)+Math.cos((initialHeading+theta)/2)*sinm/cosm2);
        }else{
            return 0;
        }
    }
    public double yOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() > time && i<pathWaypoints.length-1){
            i++;
        }
        if(i>=pathWaypoints.length-1) {
            return pathWaypoints[pathWaypoints.length - 1].getY();
        }else if(pathWaypoints[i].getType() == Waypoint.Type.CURVE_START){
            //phi is the difference in the angle between the two straight lines before and after the curve
            double phi = pathWaypoints[i].angleTo(pathWaypoints[i-1])-pathWaypoints[i].angleTo(pathWaypoints[i+1]);
            double radius = lookAheadDistance*Math.tan(phi/2);
            //theta is the angle tended by the robot in the time specified
            double theta = (Math.PI-phi)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            if(theta%Math.PI != 0) {
                return pathWaypoints[i].getY() + (radius * Math.sin(theta) / Math.cos(theta / 2)) * Math.sin((pathWaypoints[i].angleFrom(pathWaypoints[i - 1]) + theta) / 2);
            }else{
                return pathWaypoints[i].getY() + (radius*2) * Math.sin((pathWaypoints[i].angleFrom(pathWaypoints[i - 1]) + theta) / 2);
            }
        }else{
            return pathWaypoints[i].getY() + Math.sin(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)*(time-pathWaypoints[i].getTime())/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double yPrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() > time && i<pathWaypoints.length-1){
            i++;
        }
        if(i>=pathWaypoints.length-1){
            return Math.sin(pathWaypoints[pathWaypoints.length-2].angleTo(pathWaypoints[pathWaypoints.length-1]))*universalDistanceToNext(pathWaypoints.length-2)/(pathWaypoints[pathWaypoints.length-1].getTime()-pathWaypoints[pathWaypoints.length-1].getTime());
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            //phi is the difference in the angle between the two straight lines before and after the curve
            double phi = pathWaypoints[i].angleTo(pathWaypoints[i-1])-pathWaypoints[i].angleTo(pathWaypoints[i+1]);
            double radius = lookAheadDistance*Math.tan(phi/2);
            double C = (Math.PI-phi)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            //theta is the angle tended by the robot in the time specified
            double theta = C*(time-pathWaypoints[i].getTime());
            double initialHeading = pathWaypoints[i].angleFrom(pathWaypoints[i - 1]);
            double common = Math.sin(theta)/Math.cos(theta/2);
            return C*radius*((Math.cos(theta)/Math.cos(theta/2)+Math.tan(theta/2)*common/2)*Math.sin((initialHeading+theta)/2)+
                    Math.cos((initialHeading+theta)/2)*common/2);
        }else{
            return Math.sin(pathWaypoints[i].angleTo(pathWaypoints[i+1]))*universalDistanceToNext(i)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
        }
    }
    public double yDoublePrimeOfTime(double time){
        int i=0;
        while(pathWaypoints[i].getTime() > time && i<pathWaypoints.length-1){
            i++;
        }
        if(i>=pathWaypoints.length-1){
            return 0;
        }else if(pathWaypoints[i].getType()==Waypoint.Type.CURVE_START){
            //phi is the difference in the angle between the two straight lines before and after the curve
            double phi = pathWaypoints[i].angleTo(pathWaypoints[i-1])-pathWaypoints[i].angleTo(pathWaypoints[i+1]);
            double radius = lookAheadDistance*Math.tan(phi/2);
            double C = (Math.PI-phi)/(pathWaypoints[i+1].getTime()-pathWaypoints[i].getTime());
            //theta is the angle tended by the robot in the time specified
            double theta = C*(time-pathWaypoints[i].getTime());
            double initialHeading = pathWaypoints[i].angleFrom(pathWaypoints[i - 1]);
            double common = Math.sin(theta)/Math.cos(theta/2);
            double sinm = Math.sin(theta);
            double sinm2 = Math.sin(theta/2);
            double cosm = Math.cos(theta);
            double cosm2 = Math.cos(theta/2);
            return radius*C*C*(((-sinm/cosm2) + sinm2*cosm/(2*cosm2*cosm2) + (sinm+sinm2*(2*cosm*cosm2+sinm2*sinm))/(4*cosm2*cosm2*cosm2))*Math.sin((initialHeading+theta)/2))
                    + Math.cos((initialHeading+theta)/2)*(cosm/cosm2 + sinm2*sinm/(cosm2*cosm2))/2
                    + (1.0/2.0)*(Math.cos((initialHeading+theta)/2)*(cosm*cosm2+sinm*sinm2)/(cosm2*cosm2) - Math.sin((initialHeading+theta)/2)*sinm/cosm2);
        }else{
            return 0;
        }
    }

    //General Useful Functions
    public double universalDistanceToNext(int index){
        if(index+1 >= pathWaypoints.length){
            return 0;
        }
        if(pathWaypoints[index].getType() == Waypoint.Type.CURVE_START && !(index<1)){
            double phi = pathWaypoints[index].angleTo(pathWaypoints[index-1])-pathWaypoints[index].angleTo(pathWaypoints[index+1]);
            return (Math.PI-phi)*lookAheadDistance*Math.tan(phi/2);
        }else{
            return pathWaypoints[index].distanceFrom(pathWaypoints[index+1]);
        }
    }
}
