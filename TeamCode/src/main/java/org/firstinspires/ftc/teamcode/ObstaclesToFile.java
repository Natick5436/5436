package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.REV_IMU;

import java.io.File;

public class ObstaclesToFile extends LinearOpMode {

    File obstacleFile = AppUtil.getInstance().getSettingsFile("obstaclesFile.txt");
    @Override
    public void runOpMode(){
        String temp = "";
        //Field Dimensions X=2.4 meters, Y=1.8 meters
        double fieldWidthX = 2.4;
        double fieldWidthY = 1.8;
        double maxRobotWidth = 0.75;
        double pixelsPerMeter = 100.0;
        double goalX = 2.0;
        double goalY = 0.4;
        boolean[][] ob = new boolean[(int)(pixelsPerMeter*fieldWidthX)][(int)(pixelsPerMeter*fieldWidthY)];
        for(int i=0; i<ob.length; i++){
            for(int j=0; j<ob[i].length; j++){
                if((i>(int)(pixelsPerMeter*0.8128) && i<(int)(pixelsPerMeter*1.4224)) && (j>(int)(pixelsPerMeter*0.381) && j<(int)(pixelsPerMeter*0.99))){
                    ob[i][j] = true;
                }else{
                    ob[i][j] = false;
                }
            }
        }
        ReadWriteFile.writeFile(obstacleFile, "1 0 1 0 1 0");
        REV_IMU imu = new REV_IMU(this, "imu", 1, 0);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("IMU", imu.getAngle());
        }
    }
}
