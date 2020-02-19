package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Picobot Auto", group = "Autonomous")
public class PikobotAuto extends LinearOpMode {
    public DcMotor right, left, strafe;
    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        strafe = hardwareMap.dcMotor.get("strafe");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        for (int i = 0; i < 4; i++) {
            forward(0.9,2000);
            Thread.sleep(200);
            turn(1, 800, true);
            Thread.sleep(200);
        }
    }

    public void forward(double power, long time) throws InterruptedException{
        right.setPower(power);
        left.setPower(power);
        Thread.sleep(time);
        right.setPower(0);
        left.setPower(0);
    }
    public void turn(double power, long time, boolean direction) throws InterruptedException{
        if (direction) {
            left.setPower(power);
            Thread.sleep(time);
            left.setPower(0);
        }
        else {
            right.setPower(power);
            Thread.sleep(time);
            right.setPower(0);
        }
    }
}
//WaitforStart runs once, after that everything runs  one time  instead of in a while loop
//Thread.sleep is either wait(if there aren't  motors running) or it is for how long the motors move at that power