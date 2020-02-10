package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.SKYSTONE;

@Autonomous(name = "M6 Red Build", group = "Autonomous")
public class M6_Red_Build_Auto  extends LinearOpMode {
    Mark_6 robot = new Mark_6(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.initialize(hardwareMap, SKYSTONE.FIELD_WIDTH-SKYSTONE.ROBOT_WIDTH/2, SKYSTONE.FIELD_WIDTH - (SKYSTONE.WALL_TO_FOUNDATION + SKYSTONE.FOUNDATION_LENGTH/2), Math.PI, false);

        int itemSelected = 0;
        String[] settingNames = {"Move foundation: ", "Park wall side: ", "Travel wall side: ", "Help Skystone: ", "Help far side on quarry first: "};
        boolean[] settings = new boolean[settingNames.length];
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.dpad_up){
                if(itemSelected >= settingNames.length-1){
                    itemSelected = 0;
                }else{
                    itemSelected++;
                }
            }else if(gamepad1.dpad_down){
                if(itemSelected <= 0){
                    itemSelected = settingNames.length-1;
                }else{
                    itemSelected--;
                }
            }
            if(gamepad1.dpad_left|| gamepad1.dpad_right){
                settings[itemSelected] = !settings[itemSelected];
            }
            if(!settings[3]){
                settings[4] = false;
            }
            for(int i=0; i<settingNames.length; i++){
                if(i == itemSelected){
                    telemetry.addData(">>>"+settingNames[i], settings[i]);
                }else {
                    telemetry.addData(settingNames[i], settings[i]);
                }
            }
            telemetry.update();
        }
        robot.odo.start();
        runtime.reset();

    }
}
