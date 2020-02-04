package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Hardware.Mark_5;
import org.firstinspires.ftc.teamcode.Hardware.Mark_6;

@TeleOp(name = "M6 TeleOp", group = "TeleOp")
public class M6_TeleOp extends LinearOpMode {
    Mark_6 robot = new Mark_6(this);

    double drivePower;
    boolean fastMode;
    boolean yDown;
    @Override
    public void runOpMode()throws InterruptedException{
        robot.initialize(hardwareMap, 0, 0, 0,true);
        drivePower = 0.5;
        fastMode = false;
        yDown = false;

        waitForStart();
        while (opModeIsActive()) {
            //Drive System
            if(gamepad1.left_bumper){
                robot.angleStrafe(drivePower*Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
            }else if(gamepad1.right_bumper){
                robot.angleStrafe(drivePower*Math.hypot(gamepad1.right_stick_y, -gamepad1.right_stick_x), Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x));
            }else if((gamepad1.right_trigger-gamepad1.left_trigger) != 0){
                robot.strafe(drivePower*(gamepad1.right_trigger-gamepad1.left_trigger));
            }else if(gamepad1.dpad_up){
                robot.forward(drivePower);
            }else if(gamepad1.dpad_down){
                robot.forward(-drivePower);
            }else{
                robot.lF.setPower(-drivePower*gamepad1.left_stick_y);
                robot.lB.setPower(-drivePower*gamepad1.left_stick_y);
                robot.rF.setPower(-drivePower*gamepad1.right_stick_y);
                robot.rB.setPower(-drivePower*gamepad1.right_stick_y);
                robot.setStatus(Mark_5.Status.DRIVING);
            }

            //Fast mode/slow mode swap
            if(gamepad1.left_stick_button && gamepad1.right_stick_button) {
                fastMode = !fastMode;
            }
            if(fastMode){
                drivePower = 1;
                telemetry.addData("!FAST MODE!", "!HOLD YOUR HORSES! (click both stick buttons to cancel)");
            }else{
                drivePower = 0.5;
                telemetry.addData("You are in Slow mode", "(click both stick buttons to engage fast mode)");
            }
            telemetry.addData("Position", robot.rF.getCurrentPosition());
            telemetry.addData("NumThreads", Thread.activeCount());
            telemetry.addData("MaxThreads", Runtime.getRuntime().availableProcessors());
            telemetry.update();
        }
    }
}
