package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Hardware.Vuforia;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.SKYSTONE;

import java.lang.reflect.Field;

import javax.net.ssl.SSLKeyException;

@Autonomous(name = "M6 Red Build", group = "Autonomous")
public class M6_Red_Build_Auto  extends LinearOpMode {
    Mark_6 robot = new Mark_6(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        Vuforia t1 = new Vuforia(this, hardwareMap);
        t1.start();
        robot.initialize(hardwareMap, SKYSTONE.FIELD_WIDTH-SKYSTONE.ROBOT_WIDTH/2, SKYSTONE.FIELD_WIDTH-0.8, Math.PI, false);


      int itemSelected = 0;
        String[] settingNames = {"Move foundation(true = yes, false = no): ", "Park skybridge side(true) or wall side (false): ", "Travel wall side: ", "Help Skystone: ", "Help first on close side(true) or on far side (false): "};
        boolean[] settings = new boolean[settingNames.length];
        boolean active = false;
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.dpad_up && !active){
                if(itemSelected <= 0){
                    itemSelected = settingNames.length-1;
                }else{
                    itemSelected--;
                }
                active = true;
            }else if(gamepad1.dpad_down && !active){
                if(itemSelected >= settingNames.length-1){
                    itemSelected = 0;
                }else{
                    itemSelected++;
                }
                active = true;
            }else if((gamepad1.dpad_left|| gamepad1.dpad_right)&& !active){
                settings[itemSelected] = !settings[itemSelected];
                active = true;
            }else if(active&&!gamepad1.dpad_right&&!gamepad1.dpad_left&&!gamepad1.dpad_up&&!gamepad1.dpad_down){
                active = false;
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
        waitForStart();
        if(settings[0]) {
            robot.strafe(1, -0.1, true);
            robot.forward(1, -(SKYSTONE.START_TO_FOUNDATION - SKYSTONE.ROBOT_WIDTH) + 0.2, false);
            robot.forward(0.4, -0.18, false);
            robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            robot.strafe(1, -0.15, true);
            robot.arch(1, -0.21305, 0.087329, false);
            robot.forward(1, 0.5, true);
            robot.arch(1, -0.21305, 0.401987, false);
            robot.forward(1, -0.08, false);
            robot.foundation.setPosition(robot.FOUNDATION_OPEN);
        }
        telemetry.addData("Position", "X:"+robot.odo.getX()+" Y:"+robot.odo.getY());
        telemetry.update();
        if(settings[1]){
            robot.goToAbsolutePosition(1, 0.6, SKYSTONE.FIELD_WIDTH - (SKYSTONE.SKYBRIDGE_LENGTH-SKYSTONE.ROBOT_WIDTH/2), SKYSTONE.FIELD_WIDTH / 2, true);
        }else {
            robot.goToAbsolutePosition(1, 0.6, SKYSTONE.FIELD_WIDTH - SKYSTONE.ROBOT_WIDTH / 2, SKYSTONE.FIELD_WIDTH / 2, true);
        }
    }
}
