package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "M4 Red Quarry", group = "Autonomous")
@Disabled
public class M4_Red_Quarry_Auto extends LinearOpMode {
    Mark_4 robot = new Mark_4(this,hardwareMap);
    //in meters
    final double FIELD_WIDTH = 3.5814;
    final double ROBOT_WIDTH = 0.4572;
    final double DISTANCE_TO_STONES = 1.1938;
    @Override
    public void runOpMode() throws InterruptedException{

        robot.initialize(hardwareMap, FIELD_WIDTH-ROBOT_WIDTH, 1.22, Math.PI/2);
        waitForStart();
        approachStones(0.08, 0.01,0.5);
        robot.setOdometryPosition(FIELD_WIDTH-DISTANCE_TO_STONES + 0.08, 1.22);
        moveToSkystone(1);
        procureSkystone(0.4);
        robot.goToAbsolutePosition(FIELD_WIDTH-DISTANCE_TO_STONES +0.1, 2.0, 1);
        robot.turn(0.5,-Math.PI/2);
        robot.flyR.setPower(-0.5);
        robot.flyL.setPower(0.5);
        Thread.sleep(7000);
        robot.flyR.setPower(0);
        robot.flyL.setPower(0);
        while(opModeIsActive()){
            telemetry.addData("Distance: ", robot.sensorDistance.getDistance(DistanceUnit.METER));
            telemetry.addData("Skystone?: ", robot.skystone);
            telemetry.addData("OdometrX", robot.odometryX);
            telemetry.addData("OdometryY",robot.odometryY);
            telemetry.addData("OdometryAngle",robot.odometryAngle);
            telemetry.update();
        }
    }


    public void approachStones(double goalDistance, double goalDistanceAcc, double power)throws InterruptedException{
        boolean distanceReached = false;
        robot.middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Double deltaDistance = new Double(robot.sensorDistance.getDistance(DistanceUnit.METER));
        while (deltaDistance.isNaN()){
            deltaDistance = new Double(robot.sensorDistance.getDistance(DistanceUnit.METER));
            robot.middle.setPower(power);
            if (isStopRequested()){
                return;
            }
        }
        robot.middle.setPower(0);
        sleep(1000);
        robot.turn(0.05, Math.PI/2);
        while(!distanceReached){
            deltaDistance = new Double(robot.sensorDistance.getDistance(DistanceUnit.METER));
            distanceReached = false;
            //robot strafe no more
            if(deltaDistance.isNaN()){
                robot.middle.setPower(power);
            }
            if (robot.sensorDistance.getDistance(DistanceUnit.METER)-goalDistance > 0){
                robot.middle.setPower(0.25*power);
            }
            if (robot.sensorDistance.getDistance(DistanceUnit.METER)-goalDistance < 0){
                robot.middle.setPower(0.25*-power);
            }
            if(Math.abs(robot.sensorDistance.getDistance(DistanceUnit.METER) - goalDistance) < goalDistanceAcc){
                distanceReached = true;
            }
            if (distanceReached){
                robot.middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (isStopRequested()){
                return;
            }
            telemetry.addData("Distance: ", deltaDistance.doubleValue());
            telemetry.update();
        }
        robot.middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.middle.setPower(0);
        robot.turn(0.05, Math.PI/2);

    }
    public void moveToSkystone(double power) throws InterruptedException{
        while (!robot.skystone){
            Color.RGBToHSV((int) (robot.sensorColor.red() * robot.SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * robot.SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * robot.SCALE_FACTOR),
                    robot.hsvValues);
            if (robot.sensorColor.red() > 85 && robot.sensorColor.green() > 65){
                robot.skystone = false;
            }else if (robot.sensorColor.red() < 40 && robot.sensorColor.green() < 45){
                robot.skystone = true;
            }else{
                robot.skystone = false;
            }
            telemetry.addData("skystone",robot.skystone);
            telemetry.update();
            robot.right.setPower(-power);
            robot.left.setPower(-power);

            if(isStopRequested()) {
                return;
            }
        }
        robot.right.setPower(0);
        robot.left.setPower(0);
    }
    public void procureSkystone(double power) throws InterruptedException{
        robot.forwardMeters(0.2, power);
        robot.turn(power, 0);
        robot.forwardMeters(-0.3, power);
        robot.turn(power, Math.PI/2);
        robot.flyL.setPower(-0.5);
        robot.flyR.setPower(0.5);
        robot.forwardMeters(-0.3, power);
        Thread.sleep(1000);
        robot.flyL.setPower(0);
        robot.flyR.setPower(0);
    }
}
