package org.firstinspires.ftc.teamcode.Math;


import java.util.ArrayList;

public class PathInterface{

    public static boolean[][] createObstacleBuffer(boolean[][] obstacles, int robotNodeWidth){
        ArrayList<Indexes> indexes = new ArrayList<Indexes>();
        for(int x=0; x<obstacles.length; x++){
            for (int y=0; y<obstacles[x].length; y++){
                if(obstacles[x][y]){
                    indexes.add(new Indexes(x, y));
                }
            }
        }
        for(int k=0; k<indexes.size(); k++) {
            for (int i = Math.max(indexes.get(k).x - robotNodeWidth, 0); i <= Math.min(indexes.get(k).x + robotNodeWidth, obstacles.length - 1); i++) {
                for (int j = Math.max(indexes.get(k).y - robotNodeWidth, 0); j <= Math.min(indexes.get(k).y + robotNodeWidth, obstacles[i].length - 1); j++) {
                    if(Math.floor(Math.hypot(indexes.get(k).x - i, indexes.get(k).y - j)) < robotNodeWidth){
                        obstacles[i][j] = true;
                    }
                }
            }
        }
        return obstacles;
    }

    public static boolean[][] addSpaceLimits(boolean[][] obstacles){
        for(int i=0; i<obstacles.length; i++){
            obstacles[i][0] = true;
            obstacles[i][obstacles[i].length-1] = true;
        }
        for(int j=0; j<obstacles[0].length; j++){
            obstacles[0][j] = true;
            obstacles[obstacles.length-1][j] = true;
        }
        return obstacles;
    }

    public static boolean[][] drawCircle(boolean[][] obstacles, int x, int y, int width, boolean value){
        for (int i = Math.max(x - width, 0); i <= Math.min(x + width, obstacles.length - 1); i++) {
            for (int j = Math.max(y - width, 0); j <= Math.min(y + width, obstacles[i].length - 1); j++) {
                if(Math.floor(Math.hypot(x - i, y - j)) < width){
                    obstacles[i][j] = value;
                }
            }
        }
        return obstacles;
    }

    public static boolean[][] drawSquare(boolean[][] obstacles, int x, int y, int width, double angle, boolean value){
        for (int i = Math.max(x - width, 0); i <= Math.min(x + width, obstacles.length - 1); i++) {
            for (int j = Math.max(y - width, 0); j <= Math.min(y + width, obstacles[i].length - 1); j++) {
                int tempWidth = width;
                double angleFromCenter = Math.atan2(y-j, x-i);
                if(Math.abs(angleFromCenter-angle) < Math.PI/4 || Math.abs(angleFromCenter-angle) > 3*Math.PI/4){
                    tempWidth =(int)(Math.abs(width/(Math.sqrt(2.0)*Math.cos(angleFromCenter-angle))));
                }else{
                    tempWidth =(int)(Math.abs(width/(Math.sqrt(2.0)*Math.sin(angleFromCenter-angle))));
                }
                if(Math.floor(Math.hypot(x - i, y - j)) < tempWidth){
                    obstacles[i][j] = value;
                }
            }
        }
        return obstacles;
    }

    public static ArrayList<AStarSearch.Node> condense(ArrayList<AStarSearch.Node> path){
        double lastAngle;
        double nextAngle;
        for(int i=1; i<path.size()-1; i++){
            lastAngle = Math.atan2(path.get(i).getY()-path.get(i-1).getY(), path.get(i).getX()-path.get(i-1).getX());
            nextAngle = Math.atan2(path.get(i+1).getY()-path.get(i).getY(), path.get(i+1).getX()-path.get(i).getX());
            if(Math.abs(nextAngle-lastAngle) < 0.05){
                path.remove(i);
                i--;
            }
        }
        return path;
    }
    //Only use if path has been condensed
    public static ArrayList<AStarSearch.Node> findShortCuts(ArrayList<AStarSearch.Node> path, boolean[][] ob){
        for(int k=0; k<path.size()-2; k++){
            boolean pathNotClear = false;
            for(int x=path.get(k).getX(); x<path.get(k+2).getX()&& !pathNotClear; x++){
                for(int y=path.get(k).getY(); y<path.get(k+2).getY()&& !pathNotClear; y++){
                    pathNotClear = pathNotClear || ob[x][y];
                }
            }
            if(!pathNotClear){
                path.remove(k+1);
            }
        }
        return path;
    }
    public static Waypoint[] mergePoints(Waypoint[] path, double pixelDifference){
        ArrayList<Waypoint> newPath = new ArrayList<Waypoint>();
        for(int i=0; i<path.length-1; i++){
            if(Math.hypot(path[i+1].getX()-path[i].getX(), path[i+1].getY()-path[i].getY()) < pixelDifference){
                double averageX = (path[i+1].getX()+path[i].getX())/2;
                double averageY = (path[i+1].getY()+path[i].getY())/2;
                newPath.add(new Waypoint(averageX, averageY));
                i++;
            }else{
                newPath.add(path[i]);
            }
        }
        path = new Waypoint[newPath.size()];
        for(int j=0; j<newPath.size(); j++){
            path[j] = newPath.get(j);
        }
        return path;
    }
    public static Waypoint[] nodesToWaypoints(ArrayList<AStarSearch.Node> path, double metersPerNode){
        Waypoint[] newPath = new Waypoint[path.size()];
        for(int i=0; i<newPath.length; i++){
            newPath[i] = new Waypoint(path.get(i).getX()*metersPerNode, path.get(i).getY()*metersPerNode);
        }
        return newPath;
    }
    static class Indexes{
        public int x;
        public int y;
        public Indexes(int x, int y){
            this.x = x;
            this.y = y;
        }
    }
}
