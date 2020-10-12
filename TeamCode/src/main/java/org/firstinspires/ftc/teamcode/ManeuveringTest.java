package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Hardware.Mark_6;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Hardware.REV_IMU;
import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.Math.AStarSearch;
import org.firstinspires.ftc.teamcode.Math.PathInterface;
import org.firstinspires.ftc.teamcode.Math.PurePursuitPath;
import org.firstinspires.ftc.teamcode.Math.Waypoint;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.DoubleFunction;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.Odometry;
import android.graphics.*;
import android.media.Image;
import android.media.MediaCodec;
import android.widget.ImageView;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;


@Autonomous(name = "Obstacles", group = "Autonomous")
public class ManeuveringTest extends LinearOpMode {
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    BNO055IMU imu;
    double initialAngle = Math.PI/2;
    double initialX = 0.34;
    double initialY = 0.34;
    Odometry odo;
    public void runOpMode() {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 9");
        final Mecanum_Drive robot = new Mecanum_Drive(hardwareMap.dcMotor.get("lF"), hardwareMap.dcMotor.get("lB"), hardwareMap.dcMotor.get("rF"), hardwareMap.dcMotor.get("rB"), DcMotor.RunMode.RUN_USING_ENCODER, 0.05, 0.336, 0.39625, 185);
        robot.attachLinearOpMode(this);
        REV_IMU imu = new REV_IMU(this, "imu", 3, initialAngle);

        double LENGTH = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim());
        double naturalMiddleMovementPerRadian = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        odo = new Odometry(this, new DeadWheel(hardwareMap.dcMotor.get("misc")), new DeadWheel(hardwareMap.dcMotor.get("intakeR")), new DeadWheel(hardwareMap.dcMotor.get("intakeL")), Mark_6.wheelDiameter, Mark_6.ticksPerMotorRev, Mark_6.motorGearRatio, LENGTH, naturalMiddleMovementPerRadian, initialX, initialY, initialAngle, -1, 1, -1);
        odo.start();
        robot.attachAngleTracker(odo);
        robot.attachPositionTracker(odo);
        robot.enableBrakes();

        //Field Dimensions X=2.4 meters, Y=1.8 meters
        double fieldWidthX = 2.4;
        double fieldWidthY = 1.8;
        double maxRobotWidth = 0.75;
        double pixelsPerMeter = 100.0;
        double goalX = 2.0;
        double goalY = 0.4;
        boolean[][] ob = new boolean[(int)(pixelsPerMeter*fieldWidthX)][(int)(pixelsPerMeter*fieldWidthY)];
        for(int i=0; i<ob.length; i++){
            for(int j=0; j<ob[i].length; j++){
                if((i>(int)(pixelsPerMeter*0.8128) && i<(int)(pixelsPerMeter*1.4224)) && (j>(int)(pixelsPerMeter*0.381) && j<(int)(pixelsPerMeter*0.99))){
                    ob[i][j] = true;
                }else{
                    ob[i][j] = false;
                }
            }
        }
        robot.attachObstacles(ob, fieldWidthX, fieldWidthY);
        robot.createBuffers(maxRobotWidth/2);
        robot.setPathFollowingParameters(2*Math.PI, Math.PI/2, 0.25);
        telemetry.addData("Status", "Initialized, ready to go on your command");
        telemetry.update();
        waitForStart();
        /*while(Math.abs(robot.getAngle()+Math.PI/2) > 0.05){
            double angleCorrection = -Math.PI/2 - robot.getAngle();
            double m = 2*Math.PI;
            double s = Math.PI/2;
            robot.inverseKinematics(new double[]{0, 0, m*(-Math.pow(Math.E, -Math.pow(angleCorrection/(Math.sqrt(2)*s), 2))+1)*(Math.abs(angleCorrection)/angleCorrection)});
        }*/
        robot.maneuverToPosition(goalX, goalY, 0.6, 0);
        /*final double startAngle = robot.getAngle();
        robot.maneuverToPosition(initialX, initialY, 0.3, new DoubleFunction() {
            @Override
            public Double doubleFunction(double input) {
                double finishAngle = -5*Math.PI/2;
                double slope = (finishAngle-startAngle);
                return slope*input + startAngle;
            }
        });*/
    }
}
