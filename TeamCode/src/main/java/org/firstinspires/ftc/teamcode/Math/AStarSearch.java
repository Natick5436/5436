package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

public class AStarSearch {

    public enum AlgorthmType{DIJKSTRA, ASTAR}
    Node[][] map;
    ArrayList<Node> finalPath = new ArrayList<Node>();
    boolean[][] obstacles;
    private boolean solving = false;
    private int startx = -1;
    private int starty = -1;
    private int finishx = -1;
    private int finishy = -1;
    private int cellsx = 20;
    private int cellsy = 20;
    private int checks = 0;
    private int length = 0;
    private Algorithm Alg = new Algorithm();
    private int curAlg = 0;
    private LinearOpMode ln;

    public AStarSearch(LinearOpMode ln, boolean[][] obstacles, int startx, int starty, int finishx, int finishy, AlgorthmType aT){
        this.ln = ln;
        this.obstacles = obstacles;
        this.startx = startx;
        this.starty = starty;
        this.finishx = finishx;
        this.finishy = finishy;
        if(aT == AlgorthmType.DIJKSTRA){
            curAlg = 0;
        }else{
            curAlg = 1;
        }
        cellsx = obstacles.length;
        cellsy = obstacles[0].length;
        map = new Node[cellsx][cellsy];    //CREATE NEW MAP OF NODES
        for(int x = 0; x < cellsx; x++) {
            for(int y = 0; y < cellsy; y++) {
                if(!obstacles[x][y]) {
                    map[x][y] = new Node(3, x, y);    //SET ALL NODES TO EMPTY
                }else{
                    map[x][y] = new Node(2, x, y);    //Makes a wall if obstacles at [x][y] is true
                }
            }
        }
        if(startx > -1 && starty > -1) {   //RESET THE START AND FINISH
            map[startx][starty] = new Node(0,startx,starty);
            map[startx][starty].setHops(0);
        }
        if(finishx > -1 && finishy > -1)
            map[finishx][finishy] = new Node(1,finishx,finishy);
        solving = true;
        startSearch();
    }

    public void startSearch() {    //START STATE
        switch(curAlg) {
            case 0:
                Alg.Dijkstra();
                break;
            case 1:
                Alg.AStar();
                break;
        }
    }
    public ArrayList<Node> getFinalPath(){
        if(finalPath.size() == 0){
            return null;
        }else {
            return finalPath;
        }
    }
    public int numTurns(){
        if(getFinalPath() != null) {
            int numTurns = 0;
            double lastAngle = Math.atan2(getFinalPath().get(1).getY() - getFinalPath().get(0).getY(), getFinalPath().get(1).getX() - getFinalPath().get(0).getX());
            for (int i = 1; i < getFinalPath().size() - 1; i++) {
                double currentAngle = Math.atan2(getFinalPath().get(i + 1).getY() - getFinalPath().get(i).getY(), getFinalPath().get(i + 1).getX() - getFinalPath().get(i).getX());
                if (Math.abs(currentAngle - lastAngle) > 0.05) {
                    numTurns++;
                }
                lastAngle = currentAngle;
            }
            return numTurns;
        }else{
            return 0;
        }
    }

    class Algorithm {  //ALGORITHM CLASS

        //DIJKSTRA WORKS BY PROPAGATING OUTWARDS UNTIL IT FINDS THE FINISH AND THEN WORKING ITS WAY BACK TO GET THE PATH
        //IT USES A PRIORITY QUE TO KEEP TRACK OF NODES THAT IT NEEDS TO EXPLORE
        //EACH NODE IN THE PRIORITY QUE IS EXPLORED AND ALL OF ITS NEIGHBORS ARE ADDED TO THE QUE
        //ONCE A NODE IS EXLPORED IT IS DELETED FROM THE QUE
        //AN ARRAYLIST IS USED TO REPRESENT THE PRIORITY QUE
        //A SEPERATE ARRAYLIST IS RETURNED FROM A METHOD THAT EXPLORES A NODES NEIGHBORS
        //THIS ARRAYLIST CONTAINS ALL THE NODES THAT WERE EXPLORED, IT IS THEN ADDED TO THE QUE
        //A HOPS VARIABLE IN EACH NODE REPRESENTS THE NUMBER OF NODES TRAVELED FROM THE START
        public void Dijkstra() {
            ArrayList<Node> priority = new ArrayList<Node>();  //CREATE A PRIORITY QUE
            priority.add(map[startx][starty]); //ADD THE START TO THE QUE
            while(solving) {
                if(ln.isStopRequested()){
                    return;
                }
                if(priority.size() <= 0) { //IF THE QUE IS 0 THEN NO PATH CAN BE FOUND
                    solving = false;
                    break;
                }
                int hops = priority.get(0).getHops()+1;    //INCREMENT THE HOPS VARIABLE
                ArrayList<Node> explored = exploreNeighbors(priority.get(0), hops);    //CREATE AN ARRAYLIST OF NODES THAT WERE EXPLORED
                if(explored.size() > 0) {
                    priority.remove(0);    //REMOVE THE NODE FROM THE QUE
                    priority.addAll(explored); //ADD ALL THE NEW NODES TO THE QUE
                } else {   //IF NO NODES WERE EXPLORED THEN JUST REMOVE THE NODE FROM THE QUE
                    priority.remove(0);
                }
            }
        }

        //A STAR WORKS ESSENTIALLY THE SAME AS DIJKSTRA CREATING A PRIORITY QUE AND PROPAGATING OUTWARDS UNTIL IT FINDS THE END
        //HOWEVER ASTAR BUILDS IN A HEURISTIC OF DISTANCE FROM ANY NODE TO THE FINISH
        //THIS MEANS THAT NODES THAT ARE CLOSER TO THE FINISH WILL BE EXPLORED FIRST
        //THIS HEURISTIC IS BUILT IN BY SORTING THE QUE ACCORDING TO HOPS PLUS DISTANCE UNTIL THE FINISH
        public void AStar() {
            ArrayList<Node> priority = new ArrayList<Node>();
            map[startx][starty].setHops(0);
            priority.add(map[startx][starty]);
            while(solving) {
                if(ln.isStopRequested()){
                    return;
                }
                if(priority.size() <= 0) {
                    solving = false;
                    break;
                }
                int hops = priority.get(0).getHops()+1;
                ArrayList<Node> explored = exploreNeighbors(priority.get(0), hops);
                if(explored.size() > 0) {
                    priority.remove(0);
                    priority.addAll(explored);
                } else {
                    priority.remove(0);
                }
                sortQue(priority); //SORT THE PRIORITY QUE
            }
        }

        public ArrayList<Node> sortQue(ArrayList<Node> sort) { //SORT PRIORITY QUE
            int c = 0;
            while(c < sort.size()) {
                int sm = c;
                for(int i = c+1; i < sort.size(); i++) {
                    if(sort.get(i).getEuclidDist()+sort.get(i).getHops() < sort.get(sm).getEuclidDist()+sort.get(sm).getHops())
                        sm = i;
                }
                if(c != sm) {
                    Node temp = sort.get(c);
                    sort.set(c, sort.get(sm));
                    sort.set(sm, temp);
                }
                c++;
            }
            return sort;
        }

        public ArrayList<Node> exploreNeighbors(Node current, int hops) {  //EXPLORE NEIGHBORS
            ArrayList<Node> explored = new ArrayList<Node>();  //LIST OF NODES THAT HAVE BEEN EXPLORED
            for(int a = -1; a <= 1; a++) {
                for(int b = -1; b <= 1; b++) {
                    int xbound = current.getX()+a;
                    int ybound = current.getY()+b;
                    if((xbound > -1 && xbound < cellsx) && (ybound > -1 && ybound < cellsy)) { //MAKES SURE THE NODE IS NOT OUTSIDE THE GRID
                        Node neighbor = map[xbound][ybound];
                        int tempHops;
                        if(a*b == 0){
                            tempHops = current.getHops()+10;
                        }else{
                            tempHops = current.getHops()+14;
                        }
                        if((neighbor.getHops()==-1 || neighbor.getHops() > tempHops) && neighbor.getType()!=2) {   //CHECKS IF THE NODE IS NOT A WALL AND THAT IT HAS NOT BEEN EXPLORED
                            explore(neighbor, current.getX(), current.getY(), hops);   //EXPLORE THE NODE
                            explored.add(neighbor);    //ADD THE NODE TO THE LIST
                        }
                    }
                }
            }
            return explored;
        }
        public ArrayList<Node> getNeighbors(Node current, int hops) {  // NEIGHBORS
            ArrayList<Node> explored = new ArrayList<Node>();  //List of Nodes around a current node that have been checked
            for(int a = -1; a <= 1; a++) {
                for(int b = -1; b <= 1; b++) {
                    int xbound = current.getX()+a;
                    int ybound = current.getY()+b;
                    if((xbound > -1 && xbound < cellsx) && (ybound > -1 && ybound < cellsy)) { //MAKES SURE THE NODE IS NOT OUTSIDE THE GRID
                        Node neighbor = map[xbound][ybound];
                        if(neighbor.getType()==4) {    //Finds neightbors that have been explored
                            explored.add(neighbor);    //ADD THE NODE TO THE LIST
                        }
                    }
                }
            }
            return explored;
        }

        public void explore(Node current, int lastx, int lasty, int hops) {    //EXPLORE A NODE
            if(current.getType()!=0 && current.getType() != 1) //CHECK THAT THE NODE IS NOT THE START OR FINISH
                current.setType(4);    //SET IT TO EXPLORED

            current.setLastNode(lastx, lasty); //KEEP TRACK OF THE NODE THAT THIS NODE IS EXPLORED FROM
            if((current.getY()-lasty)*(current.getX()-lastx) == 0) {
                current.setHops(map[lastx][lasty].getHops() + 10);    //SET THE HOPS FROM THE START
            }else{
                current.setHops(map[lastx][lasty].getHops() + 14);
            }
            checks++;
            if(current.getType() == 1) {   //IF THE NODE IS THE FINISH THEN BACKTRACK TO GET THE PATH
                backtrack(current);
            }
        }

        public void backtrack(Node start) {    //BACKTRACK
            length = start.getHops();
            while(!(start.getLastX() == startx && start.getLastY() == starty)) {   //BACKTRACK FROM THE END OF THE PATH TO THE START
                start = map[start.getLastX()][start.getLastY()];
                start.setType(5);
                finalPath.add(0, start);
            }
            solving = false;
        }
    }

    public class Node {

        // 0 = start, 1 = finish, 2 = wall, 3 = empty, 4 = checked, 5 = finalpath
        private int cellType = 0;
        private int hops;
        private int x;
        private int y;
        private int lastX;
        private int lastY;
        private double dToEnd = 0;

        public Node(int type, int x, int y) {  //CONSTRUCTOR
            cellType = type;
            this.x = x;
            this.y = y;
            hops = -1;
        }

        public double getEuclidDist() {       //CALCULATES THE EUCLIDIAN DISTANCE TO THE FINISH NODE
            int xdif = Math.abs(x-finishx);
            int ydif = Math.abs(y-finishy);
            dToEnd = Math.sqrt((xdif*xdif)+(ydif*ydif));
            return dToEnd;
        }

        public int getX() {return x;}     //GET METHODS
        public int getY() {return y;}
        public int getLastX() {return lastX;}
        public int getLastY() {return lastY;}
        public int getType() {return cellType;}
        public int getHops() {return hops;}

        public void setType(int type) {cellType = type;}      //SET METHODS
        public void setLastNode(int x, int y) {lastX = x; lastY = y;}
        public void setHops(int hops) {this.hops = hops;}
    }
}
