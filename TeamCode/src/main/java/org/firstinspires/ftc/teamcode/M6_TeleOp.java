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
    boolean aDown;
    boolean xDown;
    boolean sticksDown;
    boolean sticksPressed;
    boolean manual  = false;
    boolean liftLimit = false;
    boolean rightBumper = false;
    int encoderInit = 0;

    @Override
    public void runOpMode()throws InterruptedException{
        robot.initialize(hardwareMap, 0, 0, 0,false);
        drivePower = 0.5;
        fastMode = false;
        yDown = false;
        boolean armClose = true;
        aDown = false;
        boolean clampClose = true;
        boolean foundationClose = true;
        xDown = false;
        boolean bumperDown = false;

        waitForStart();
        robot.odo.start();
        while (opModeIsActive()) {
            //Drive System
            if(gamepad1.left_bumper){
                robot.angleStrafe(0.25*Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
                bumperDown = true;
            }else if(gamepad1.right_bumper){
                robot.angleStrafe(0.25*Math.hypot(gamepad1.right_stick_y, -gamepad1.right_stick_x), Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x));
                bumperDown = true;
            }else if((gamepad1.right_trigger-gamepad1.left_trigger) != 0){
                robot.strafe(drivePower*(gamepad1.right_trigger-gamepad1.left_trigger));
            }else{
                if(!bumperDown) {
                    robot.lF.setPower(-drivePower * gamepad1.left_stick_y);
                    robot.lB.setPower(-drivePower * gamepad1.left_stick_y);
                    robot.rF.setPower(-drivePower * gamepad1.right_stick_y);
                    robot.rB.setPower(-drivePower * gamepad1.right_stick_y);
                    robot.setStatus(Mark_5.Status.DRIVING);
                }else{
                    if(Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x) < 0.1 && Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) < 0.1){
                        bumperDown = false;
                    }
                }
            }
            //Fast mode/slow mode swap
            if (gamepad1.dpad_up){
                fastMode = true;
            }
            if(gamepad1.dpad_down){
                fastMode = false;
            }
            /*if(gamepad1.left_stick_button && gamepad1.right_stick_button && !sticksDown) {
                fastMode = !fastMode;
                sticksDown = true;
            }else if(!(gamepad1.left_stick_button && gamepad1.right_stick_button)){
                sticksDown = false;
            }*/
            if (gamepad2.start){
                encoderInit = -robot.lift.getCurrentPosition()-1;
                telemetry.addData("Init Encoder",encoderInit);
            }
            if(fastMode){
                drivePower = 1;
                telemetry.addData("!FAST MODE!", "!HOLD YOUR HORSES! (click both stick buttons to cancel)");
            }else{
                drivePower = 0.5;
                telemetry.addData("You are in Slow mode", "(click both stick buttons to engage fast mode)");
            }

            //Intake control
            robot.intakeL.setPower(gamepad2.left_stick_y);
            robot.intakeR.setPower(gamepad2.right_stick_y);

            if (gamepad2.x){
                robot.lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            }else if (!manual) {
                if (robot.lift.getCurrentPosition() + encoderInit < 0 && robot.lift.getCurrentPosition() + encoderInit > -20000) {
                    liftLimit = false;
                    robot.lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                } else {
                    liftLimit = true;
                    robot.lift.setPower(0);
                    if (robot.lift.getCurrentPosition() + encoderInit >= 0) {
                        robot.lift.setPower(-gamepad2.right_trigger);
                    } else if (robot.lift.getCurrentPosition() <= -20000) {
                        robot.lift.setPower(gamepad2.left_trigger);
                    }
                }
            }

            if(gamepad2.dpad_up){
                robot.extension.setPower(1);
            }else if(gamepad2.dpad_down){
                robot.extension.setPower(-1);
            }else{
                robot.extension.setPower(0);
            }
            if(gamepad2.right_bumper){
                robot.outClamp.setPosition(robot.OUT_GRAB);
            }else if(gamepad2.left_bumper){
                robot.outClamp.setPosition(robot.OUT_RELEASE);
            }
            /*if(gamepad2.dpad_up){
                robot.extRotate.setPosition(robot.ROTATE_OUT);
            }else if(gamepad2.dpad_down){
                robot.extRotate.setPosition(robot.ROTATE_IN);
            }else if(gamepad2.dpad_right || gamepad2.dpad_left){
                robot.extRotate.setPosition(robot.ROTATE_MID);
            }*/

            //Misc servo manips control
            if(!yDown && gamepad1.y){
                armClose = !armClose;
                yDown= true;
            }else if(!gamepad1.y){
                yDown = false;
            }
            if(armClose){
                robot.skyArm2.setPosition(1-robot.SKYARM_DOWN);

            }else{
                robot.skyArm2.setPosition(1-robot.SKYARM_UP);
            }
            if (gamepad1.b){
                robot.skyClamp1.setPosition(robot.SKYCLAMP_CLOSE);
                robot.skyClamp2.setPosition(robot.SKYCLAMP_CLOSE);
            }
            if (gamepad1.x){
                robot.skyClamp1.setPosition(robot.SKYCLAMP_OPEN);
                robot.skyClamp2.setPosition(robot.SKYCLAMP_OPEN);
            }
            if(!aDown && gamepad1.a){
                clampClose = !clampClose;
                aDown = true;
            }else if(!gamepad1.a){
                aDown = false;
            }
            if (clampClose){
                robot.skyArm1.setPosition(robot.SKYARM_DOWN);
            }else {
                robot.skyArm1.setPosition(robot.SKYARM_UP);
            }
            if (!xDown && gamepad1.x){
                foundationClose = !foundationClose;
                xDown = true;
            }else if(!gamepad1.x){
                xDown = false;
            }
            if (foundationClose){
                robot.foundation.setPosition(robot.FOUNDATION_CLOSE);
            }else{
                robot.foundation.setPosition(robot.FOUNDATION_OPEN);
            }
            //a = in, b= 90 deg, y = close
            if (gamepad2.a){
                robot.extRotate.setPosition(robot.ROTATE_IN);
            }
            if (gamepad2.b){
                robot.extRotate.setPosition(robot.ROTATE_MID);
            }
            if (gamepad2.y){
                robot.extRotate.setPosition(robot.ROTATE_OUT);
            }

            telemetry.addData("Velocities", "left: "+robot.odo.getVelocityL()+"    right: "+robot.odo.getVelocityR());
            telemetry.addData("left dead wheel:", robot.odo.left.getCurrentPosition());
            telemetry.addData("right dead wheel: ", robot.odo.right.getCurrentPosition());
            telemetry.addData("middle dead wheel: ", robot.odo.middle.getCurrentPosition());
            telemetry.addData("Position", "X: "+robot.odo.getX()+"Y: "+robot.odo.getY()+"Angle: "+robot.odo.getAngle());
            telemetry.addData("Heading", robot.getHeading());
            //telemetry.addData("Distance to Object",robot.getCurrentDistance());
            telemetry.addData("Lift Limit",liftLimit);
            telemetry.addData("Lift Position",robot.lift.getCurrentPosition());
            telemetry.update();
        }
    }
}
