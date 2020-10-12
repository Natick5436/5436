package org.firstinspires.ftc.teamcode;

import android.view.View;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

@TeleOp(name = "Manipulation of Android", group = "TeleOp")
public class AndroidStudioManipulation extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                FtcRobotControllerActivity.activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        FtcRobotControllerActivity.setImageVisibility(View.VISIBLE);
                    }
                });
            }
            if(gamepad1.b){
                FtcRobotControllerActivity.activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        FtcRobotControllerActivity.setImageVisibility(View.INVISIBLE);
                    }
                });
            }
        }
    }
}
