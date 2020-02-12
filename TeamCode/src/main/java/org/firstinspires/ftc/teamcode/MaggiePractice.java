package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Mark_4;

@TeleOp(name = "Maggie's Practice", group = "TeleOp")

public class MaggiePractice extends LinearOpMode {
public DcMotor right,left,strafe;
    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        strafe = hardwareMap.dcMotor.get("strafe");

        waitForStart();

        while (opModeIsActive()) {
            right.setPower(gamepad1.right_stick_y);
            left.setPower(gamepad1.left_stick_y);
            if (gamepad1.right_trigger > 0.5){
                strafe.setPower(1);
            }else{
                if (gamepad1.left_trigger > 0.5){
                    strafe.setPower(-1);
                }else{
                    strafe.setPower(0);
                }
            }
        }
    }
}