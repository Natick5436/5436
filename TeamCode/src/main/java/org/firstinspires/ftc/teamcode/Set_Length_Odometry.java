package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "LENGTH SETTING", group = "Autonomous")
public class Set_Length_Odometry extends LinearOpMode {
    Gilgamesh gilgamesh = new Gilgamesh(this);
    @Override
    public void runOpMode(){
        gilgamesh.initialize(hardwareMap, 0, 0, 0);
        telemetry.addData("Ready", "Ready");
        telemetry.update();

        waitForStart();
        gilgamesh.updateOdometryData();
        gilgamesh.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gilgamesh.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(gilgamesh.sensorGyro.getHeading() != 90){
            if(gilgamesh.sensorGyro.getHeading() < 90){
                gilgamesh.left.setPower(0.05);
                gilgamesh.right.setPower(-0.05);
            }else if(gilgamesh.sensorGyro.getHeading() > 90){
                gilgamesh.left.setPower(-0.2);
                gilgamesh.right.setPower(0.2);
            }
            gilgamesh.updateOdometryData();
            if (isStopRequested()){
                return;
            }
        }
        gilgamesh.left.setPower(0);
        gilgamesh.right.setPower(0);
        while(gilgamesh.sensorGyro.getHeading() != 90){
            if(gilgamesh.sensorGyro.getHeading() < 90){
                gilgamesh.left.setPower(0.1);
                gilgamesh.right.setPower(-0.1);
            }else if(gilgamesh.sensorGyro.getHeading() > 90){
                gilgamesh.left.setPower(-0.1);
                gilgamesh.right.setPower(0.1);
            }else {
                gilgamesh.left.setPower(0);
                gilgamesh.right.setPower(0);
            }
                gilgamesh.updateOdometryData();
            if (isStopRequested()){
                return;
            }

        }
        gilgamesh.left.setPower(0);
        gilgamesh.right.setPower(0);
        while(gilgamesh.sensorGyro.getHeading() != 90){
            if(gilgamesh.sensorGyro.getHeading() < 90){
                gilgamesh.left.setPower(0.05);
                gilgamesh.right.setPower(-0.05);
            }else if(gilgamesh.sensorGyro.getHeading() > 90){
                gilgamesh.left.setPower(-0.05);
                gilgamesh.right.setPower(0.05);
            }else{
                gilgamesh.left.setPower(0);
                gilgamesh.right.setPower(0);
            }
            gilgamesh.updateOdometryData();
            if (isStopRequested()){
                return;
            }
        }
        gilgamesh.left.setPower(0);
        gilgamesh.right.setPower(0);
        while(opModeIsActive()){
            telemetry.addData("Odometry X", gilgamesh.odometryX);
            telemetry.addData("Odometry Y", gilgamesh.odometryY);
            telemetry.addData("Odometry Angle", gilgamesh.odometryAngle);
            telemetry.addData("Gyro Angle", gilgamesh.sensorGyro.getHeading());
            telemetry.addData("Current Encoder Left", gilgamesh.currentEncoderL);
            telemetry.addData("Current Encoder Right", gilgamesh.currentEncoderL);
            telemetry.update();
        }
    }
}
