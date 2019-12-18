package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mark Four TeleOp", group = "TeleOp")
@Disabled
public class Mark_Four_TeleOp extends LinearOpMode {
    private Mark_4 robot = new Mark_4(this, hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap, 0, 0, 0);

        waitForStart();

        while (opModeIsActive()) {

            robot.flyL.setPower(-0.5*gamepad2.left_stick_y);
            robot.flyR.setPower(0.5*gamepad2.right_stick_y);

            if (gamepad2.dpad_up){
                robot.extension.setPower(0.5);
            }else if(gamepad2.dpad_down){
                robot.extension.setPower(-0.5);
            }else{
                robot.extension.setPower(0);
            }

            robot.right.setPower(gamepad1.right_stick_y);
            robot.left.setPower(gamepad1.left_stick_y);

            /*if (gamepad2.a) {
                robot.grab.setPosition(0.4);
            }
            if (gamepad2.y){
                robot.grab.setPosition(0.75);
            }*/
            if (gamepad1.a) {
               robot.hook.setPosition(0.4);
            }
            if (gamepad1.y) {
                robot.hook.setPosition(0.65);
            }
            if(gamepad2.a){
                robot.grab.setPosition(0.0);
            }
            if(gamepad2.b){
                robot.grab.setPosition(0.5);
            }

            robot.middle.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            robot.liftL.setPower(0.3*(gamepad2.right_trigger - gamepad2.left_trigger));
            robot.liftR.setPower(0.3*(gamepad2.left_trigger - gamepad2.right_trigger));
            telemetry.addData("DeadWheel Encoder", robot.deadWheel.getCurrentPosition());
            telemetry.addData("Deadwheel distance in meters", robot.deadWheel.getCurrentPosition() * robot.odometryWheelCirc / robot.ticksPerOdometryWheel);
            telemetry.addData("Angle Heading", robot.getHeading());
            telemetry.update();
        }
    }
}