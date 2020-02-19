package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Picobot", group = "TeleOp")
public class PikobotTeleOp extends LinearOpMode {
    public DcMotor right, left, strafe;
    @Override
    public void runOpMode() {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        strafe = hardwareMap.dcMotor.get("strafe");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            right.setPower(-gamepad1.right_stick_y);
            left.setPower(-gamepad1.left_stick_y);
            strafe.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }

    }
}