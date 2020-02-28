package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Disabled
@TeleOp(name = "Picobot", group = "TeleOp")
public class PikobotTeleOp extends LinearOpMode {
    public DcMotor right, left, strafe, arm, clamp;
    private boolean pressed = true;

    @Override
    public void runOpMode() {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        strafe = hardwareMap.dcMotor.get("strafe");
        arm = hardwareMap.dcMotor.get("arm");
        clamp = hardwareMap.dcMotor.get("clamp");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        //This is a sound file of Homer Simpson screaming
        int death = hardwareMap.appContext.getResources().getIdentifier("aaaahh", "raw", hardwareMap.appContext.getPackageName());

        int honk = hardwareMap.appContext.getResources().getIdentifier("honk", "raw", hardwareMap.appContext.getPackageName());

        waitForStart();
        while(opModeIsActive()) {
            right.setPower(-gamepad1.right_stick_y);
            left.setPower(-gamepad1.left_stick_y);
            strafe.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            if ((gamepad1.a || gamepad1.x) && pressed) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, death);
                pressed = false;
            }
            if (gamepad1.dpad_up){
                arm.setPower(0.2);
            }else if (gamepad1.dpad_down){
                arm.setPower(-0.2);
            }else{
                arm.setPower(0);
            }
            if (gamepad1.dpad_right){
                clamp.setPower(0.15);
            }else if (gamepad1.dpad_left){
                clamp.setPower(-0.15);
            }else{
                clamp.setPower(0);
            }
        }
    }

}