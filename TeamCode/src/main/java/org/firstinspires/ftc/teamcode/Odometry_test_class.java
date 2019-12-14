package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Gilgamesh Odometry Test", group = "TeleOp")
public class Odometry_test_class extends LinearOpMode{
    Gilgamesh gilgamesh = new Gilgamesh(this);
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException{
        gilgamesh.initialize(hardwareMap, 0, 0, 0);
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            gilgamesh.left.setPower(gamepad1.left_stick_y);
            gilgamesh.right.setPower(gamepad1.right_stick_y);
            gilgamesh.updateOdometryData();
            telemetry.addData("X Position", gilgamesh.odometryX);
            telemetry.addData("Y Position", gilgamesh.odometryY);
            telemetry.addData("Angle", gilgamesh.odometryAngle);
            telemetry.addData("Gyro Angle", Math.toRadians(gilgamesh.sensorGyro.getHeading()));
            telemetry.update();
        }
    }
}
