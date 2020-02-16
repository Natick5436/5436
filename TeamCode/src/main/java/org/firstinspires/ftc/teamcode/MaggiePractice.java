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

@TeleOp(name = "Picobot", group = "TeleOp")
public class MaggiePractice extends LinearOpMode {
public DcMotor right,left,strafe;
    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left = hardwareMap.dcMotor.get("left");
        strafe = hardwareMap.dcMotor.get("strafe");

        waitForStart();

        while (opModeIsActive()) {
            right.setPower(-gamepad1.right_stick_y);
            left.setPower(-gamepad1.left_stick_y);
            strafe.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        }
    }
}