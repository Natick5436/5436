package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mark 5 Test", group = "TeleOp")
public class Mark_5_Test extends LinearOpMode {
   Mark_5 robot = new Mark_5(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, 0, 0, 0);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("arm pos", robot.encoderCount);
            if (robot.encoderCount > -1260){
                robot.flip.setPosition(1);
            }
            if (robot.encoderCount < -1260){
                robot.flip.setPosition(0.33);
            }
            if (gamepad2.dpad_left){
                //while(robot.encoderCount < -300){
                    robot.arm.setPower(0.2);
                    if (isStopRequested()){return;}
                }
                robot.arm.setPower(0);

            //}
            if (gamepad2.dpad_up){
                robot.arm.setPower(0.3);
                robot.encoderCount = robot.arm.getCurrentPosition();
                telemetry.addData("arm position", robot.encoderCount);
            }else if (gamepad2.dpad_down){
                robot.arm.setPower(-0.3);
                robot.encoderCount = robot.arm.getCurrentPosition();
                telemetry.addData("arm position", robot.encoderCount);
            } else {
                robot.arm.setPower(0.0);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (gamepad2.right_bumper){
                robot.clamp.setPosition(0.45);
            }
            if (gamepad2.left_bumper){
                robot.clamp.setPosition(0.9);
            }

            //Drive System
            if(gamepad1.left_bumper){
                robot.strafe(-1);
            }else if(gamepad1.right_bumper){
                robot.strafe(1);
            }else{
                robot.lF.setPower(0.5*gamepad1.left_stick_y);
                robot.lB.setPower(0.5*gamepad1.left_stick_y);
                robot.rF.setPower(0.5*gamepad1.right_stick_y);
                robot.rB.setPower(0.5*gamepad1.right_stick_y);
                robot.setStatus(Mark_5.Status.DRIVING);
            }
            robot.updateVuforia();
            telemetry.update();
        }
    }
}

