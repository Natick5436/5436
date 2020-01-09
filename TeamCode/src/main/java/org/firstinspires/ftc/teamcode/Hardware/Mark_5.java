package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Math.ACMath;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Mark_5 {
    public enum Status {STRAFING, DRIVING, TURNING, ANGLE_STRAFING, INITIALIZING, INITIALIZED, PREINIT}
    private Status robotStatus;
    public void setStatus(Status s){
        robotStatus = s;
    }
    public Status getStatus(){
        return robotStatus;
    }

    public DcMotor liftL, liftR, arm, lF, lB, rF, rB;
    public Servo flip, clamp, grabL, grabR, extensionL, extensionR, stoneL, stoneR;

    public int encoderCount = 0;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Ab8+W6//////AAABmR3H1hyVGUZVv4x9ZpH2zrNEztLMVpPNP03jwLczpJ2S0BriANfEdDDjqI1OkjXuWWcW5B"
        +"x4Vwj61Z02XWaH0weIlqerhthLUbjHAW+c09rkufj1cT3q7lR+hAx3WHpK8z1dKI//BHSVWPJVaOYRKBxFIV7D2na"
        +"YDqUPv7ohd2aj8veWRF8Kpb5NjYHS80zy7uBmnFh4Y8zkZLBpzR1KVGePagHGXJNL412r+jXIgJLtqTD7v/9OFAwy"
        +"a/XaloNrsFhtq3/Kho5uJVUkIX6BQbVWLtXW/IvrPbkLzGQqlS9hPz/5t2Arp9IFg884z/d10vw5DMW9ntxXOF3PI"
        +"Ue18kWFmJEKjJ4Y+BouS9LhL8MU";

    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 180;
    private float phoneZRotate    = 0;

    List<VuforiaTrackable> allTrackables;
    public VuforiaTrackables targetsSkyStone;

    private boolean skystone;
    private VectorF skystonePosition;
    private Orientation skystoneOrientation;

    BNO055IMU imu;
    Orientation angles;

    //in meters or radians respectively unless specified
    public double odometryX;
    public double odometryY;
    public double odometryAngle, initialAngle;
    double lastEncoderR, lastEncoderL;
    double currentEncoderL, currentEncoderR;

    //wheel Diameter in cm
    final double wheelDiameter = 10.0;
    //wheel Circumference in meters
    final double wheelCirc = wheelDiameter * Math.PI/100;
    //ticks per full revolution of the motor axle
    final double ticksPerMotorRev = 537.6;
    final double motorGearRatio = 1.0;
    //ticks per full revolution of the wheel
    final double ticksPer = motorGearRatio*ticksPerMotorRev;

    DcMotor deadWheel;
    final double ticksPerOdometryWheel = 1430;
    //diameter in cm
    final double odometryWheelDiameter = 7.62;
    final double odometryWheelCirc = odometryWheelDiameter * Math.PI / 100;

    final double angleAccuracy = 0.005;
    final double distanceAccuracy = 0.01;
    final int armAccuracy = 10;

    //Init methods
    LinearOpMode ln;
    public Mark_5(LinearOpMode linear){
        ln = linear;
        robotStatus = Status.PREINIT;
        skystone = false;
        skystonePosition= new VectorF(100,100,100);
    }
    public void initialize(HardwareMap hardwareMap, double startX, double startY, double startAngle, boolean teleOpInit){
        this.setStatus(Status.INITIALIZING);
      
        //***Main robot init***
        lF = hardwareMap.dcMotor.get("lF");
        lB = hardwareMap.dcMotor.get("lB");
        rF = hardwareMap.dcMotor.get("rF");
        rB = hardwareMap.dcMotor.get("rB");

        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);

        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.dcMotor.get("arm");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftL.setDirection(DcMotorSimple.Direction.REVERSE);

        flip = hardwareMap.servo.get("flip");
        clamp = hardwareMap.servo.get("clamp");
        grabL = hardwareMap.servo.get("grabL");
        grabR = hardwareMap.servo.get("grabR");
        extensionL = hardwareMap.servo.get("eL");
        extensionR = hardwareMap.servo.get("eR");
        stoneL = hardwareMap.servo.get("stoneL");
        stoneR = hardwareMap.servo.get("stoneR");

        grabR.setDirection(Servo.Direction.REVERSE);
        extensionR.setDirection(Servo.Direction.REVERSE);
        stoneR.setDirection(Servo.Direction.REVERSE);

        if(teleOpInit){
            grabL.setPosition(1);
            grabR.setPosition(1);
            extensionL.setPosition(0.84);
            extensionR.setPosition(0.85);
            stoneL.setPosition(1);
            stoneR.setPosition(1);
            clamp.setPosition(0.45);
            flip.setPosition(0.33);
        }else {
            grabL.setPosition(0);
            grabR.setPosition(0);
            extensionL.setPosition(0.98);
            extensionR.setPosition(0.97);
            stoneL.setPosition(0.6);
            stoneR.setPosition(0.6);
            clamp.setPosition(0.45);
            flip.setPosition(0.33);
        }
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //***Odometry init***
        odometryX = startX;
        odometryY = startY;
        odometryAngle = ACMath.toStandardAngle(startAngle);
        initialAngle = ACMath.toStandardAngle(startAngle);

        deadWheel = hardwareMap.dcMotor.get("deadWheel");
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //***Vuforia init***
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");


        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            if(ln.isStopRequested())return;
        }
        targetsSkyStone.activate();*/

        //***IMU init***
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        while(imu.isGyroCalibrated()){
            if(ln.isStopRequested()){
                return;
            }
            if(ln.isStarted()){
                return;
            }
        }
        this.setStatus(Status.INITIALIZED);
        ln.telemetry.addData("Status","Ready");
        ln.telemetry.update();
    }

    //Vuforia methods
   public void updateVuforia(){
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        skystone = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                if (trackable.getName().equals("Stone Target")){
                    skystone = true;
                }else {
                    skystone = false;
                }
                ln.telemetry.addData("Visible Target", trackable.getName());
                ln.telemetry.addData("isSkystone", skystone);
                targetVisible = true;


                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            skystonePosition = lastLocation.getTranslation();
            ln.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            skystoneOrientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            ln.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            ln.telemetry.addData("Visible Target", "none");
        }
    }
    public boolean isSkystone(){
        updateVuforia();
        return skystone;
    }
    public VectorF getSkystonePosition(){
        updateVuforia();
        return skystonePosition;
    }
    public Orientation getSkystoneOrientation(){
        updateVuforia();
        return skystoneOrientation;
    }
    //Odometry methods
    public void updateLinearOdometryData(){
        odometryAngle = getHeading();
        currentEncoderL = lF.getCurrentPosition();
        currentEncoderR = rF.getCurrentPosition();
        double deltaTickL = currentEncoderL - lastEncoderL;
        double deltaTickR = currentEncoderR - lastEncoderR;
        double dL = wheelCirc * deltaTickL / ticksPer;
        double dR = wheelCirc * deltaTickR / ticksPer;
        double dM = (dL + dR) / 2;
        odometryX = odometryX + (dM * Math.cos(odometryAngle));
        odometryY = odometryY + (dM * Math.sin(odometryAngle));
        lastEncoderL = currentEncoderL;
        lastEncoderR = currentEncoderR;
    }
    public void setOdometryPosition(double x, double y){
        odometryX = x;
        odometryY = y;
    }

    //Movement methods
    double startAngle;
    final double percentCorrection = 0;
    final double maxCorrectionAngle = Math.PI/12;
    public void forward(double power){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if(getStatus() != Status.DRIVING){
            startAngle = getHeading();
        }
        this.setStatus(Status.DRIVING);
        double turnOffset;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }else{
            turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }
        lF.setPower(power+turnOffset);
        lB.setPower(power+turnOffset);
        rF.setPower(power-turnOffset);
        rB.setPower(power-turnOffset);
        updateLinearOdometryData();
    }
    //this function only takes inputs from the front wheels of the robot
    public void forward(double power, double meters){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if(getStatus() != Status.DRIVING){
            startAngle = getHeading();
        }
        double targetTickDelta = (meters / wheelCirc)*ticksPer;
        double targetTickR = targetTickDelta+rF.getCurrentPosition();
        double targetTickL = targetTickDelta+lF.getCurrentPosition();
        double deltatickR;
        double deltatickL;
        this.setStatus(Status.DRIVING);
        double turnOffset;
        //(Math.abs(rF.getCurrentPosition()-targetTickR) + Math.abs(lF.getCurrentPosition()-targetTickL))/2 >= (distanceAccuracy/wheelCirc)*ticksPer
        while ((Math.abs(rF.getCurrentPosition()-targetTickR) + Math.abs(lF.getCurrentPosition()-targetTickL))/2 >= (distanceAccuracy/wheelCirc)*ticksPer) {
            if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }else{
                turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }
            deltatickL = targetTickL - lF.getCurrentPosition();
            deltatickR = targetTickR - rF.getCurrentPosition();
            if(rF.getCurrentPosition() != targetTickR) {
                rF.setPower((power * (Math.abs(deltatickR) / deltatickR))-turnOffset);
                rB.setPower((power * (Math.abs(deltatickR) / deltatickR))-turnOffset);
            }else{
                rF.setPower(0);
                rB.setPower(0);
            }
            if (lF.getCurrentPosition() != targetTickL) {
                lF.setPower((power * (Math.abs(deltatickL) / deltatickL))+turnOffset);
                lB.setPower((power * (Math.abs(deltatickL) / deltatickL))+turnOffset);
            }else{
                lF.setPower(0);
                lB.setPower(0);
            }
            ln.telemetry.addData("left encoder", lF.getCurrentPosition());
            ln.telemetry.addData("right encoder", rF.getCurrentPosition());
            ln.telemetry.addData("targetTickL", targetTickL);
            ln.telemetry.addData("targetTickR", targetTickR);
            ln.telemetry.update();
            if(ln.isStopRequested())return;
            updateLinearOdometryData();
        }
        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }
    public void turn(double power){
        this.setStatus(Status.TURNING);
        rF.setPower(power);
        rB.setPower(power);
        lF.setPower(-power);
        lB.setPower(-power);
    }
    public void turn(double power, double targetAngle){
        odometryAngle = getHeading();
        double angle = odometryAngle;
        double targetAngleDelta;
        boolean useCompassAngle = ACMath.compassAngleShorter(targetAngle, angle);
        if(useCompassAngle) {
            targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
        }else{
            targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
        }
        double targetAngleAbs = Math.abs(targetAngleDelta);
        boolean sw = false;
        if (targetAngleDelta > 0)
            sw = true;
        if (targetAngleDelta < 0)
            sw = false;
        int decreaseRate = 0;
        while (targetAngleAbs >= angleAccuracy){
            //If change is negative, robot turns in negative direction
            if (targetAngleDelta < 0){
                //Turn right
                rF.setPower(-power / decreaseRate);
                rB.setPower(-power / decreaseRate);
                lF.setPower(power / decreaseRate);
                lB.setPower(power / decreaseRate);
                if(!sw){
                    decreaseRate++;
                }
                sw = true;
            }
            if (targetAngleDelta > 0){
                //Turn left
                lF.setPower(-power /  decreaseRate);
                lB.setPower(-power /  decreaseRate);
                rF.setPower(power / decreaseRate);
                rB.setPower(power / decreaseRate);
                if(sw){
                    decreaseRate++;
                }
                sw = false;
            }
            odometryAngle = getHeading();
            angle = odometryAngle;
            if(useCompassAngle) {
                targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
            }else{
                targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
            }
            targetAngleAbs = Math.abs(targetAngleDelta);
            if(ln.isStopRequested())return;
            ln.telemetry.addData("targetAngleDelta", targetAngleDelta);
            ln.telemetry.addData("odometryAngle", angle);
            ln.telemetry.addData("targetAngle", targetAngle);
            ln.telemetry.addData("Compass Angle: ", useCompassAngle);
            ln.telemetry.update();
        }
        rF.setPower(0);
        rB.setPower(0);
        lF.setPower(0);
        lB.setPower(0);
    }
    public void turn(double power, double targetAngle,boolean override){
        odometryAngle = getHeading();
        double angle = odometryAngle;
        double targetAngleDelta;
        if(override) {
            targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
        }else{
            targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
        }
        double targetAngleAbs = Math.abs(targetAngleDelta);
        boolean sw = false;
        if (targetAngleDelta > 0)
            sw = true;
        if (targetAngleDelta < 0)
            sw = false;
        int decreaseRate = 0;
        while (targetAngleAbs >= angleAccuracy){
            if (targetAngleDelta < 0){
                rF.setPower(-power / decreaseRate);
                rB.setPower(-power / decreaseRate);
                lF.setPower(power / decreaseRate);
                lB.setPower(power / decreaseRate);
                if(!sw){
                    decreaseRate++;
                }
                sw = true;
            }
            if (targetAngleDelta > 0){
                lF.setPower(-power /  decreaseRate);
                lB.setPower(-power /  decreaseRate);
                rF.setPower(power / decreaseRate);
                rB.setPower(power / decreaseRate);
                if(sw){
                    decreaseRate++;
                }
                sw = false;
            }
            odometryAngle = getHeading();
            angle = odometryAngle;
            if(override) {
                targetAngleDelta = ACMath.toCompassAngle(targetAngle) - ACMath.toCompassAngle(angle);
            }else{
                targetAngleDelta = ACMath.toStandardAngle(targetAngle) - ACMath.toStandardAngle(angle);
            }
            targetAngleAbs = Math.abs(targetAngleDelta);
            if(ln.isStopRequested())return;
            ln.telemetry.addData("targetAngleDelta", targetAngleDelta);
            ln.telemetry.addData("odometryAngle", angle);
            ln.telemetry.addData("targetAngle", targetAngle);
            ln.telemetry.addData("abs",targetAngleAbs);
            ln.telemetry.update();
        }
        rF.setPower(0);
        rB.setPower(0);
        lF.setPower(0);
        lB.setPower(0);
    }

    public void strafe(double power) {
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if (getStatus() != Status.STRAFING) {
            startAngle = getHeading();
        }
        this.setStatus(Status.STRAFING);
        double turnOffset;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }else{
            turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }
        lF.setPower(power+turnOffset);
        lB.setPower(-power+turnOffset);
        rF.setPower(-power-turnOffset);
        rB.setPower(power-turnOffset);
    }
    public void strafe(double power, double distance){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if (!(robotStatus == Status.STRAFING)) {
            startAngle = getHeading();
        }
        this.setStatus(Status.STRAFING);
        double turnOffset;
        while(Math.abs(distance-(deadWheel.getCurrentPosition()*odometryWheelCirc/ticksPerOdometryWheel)) > distanceAccuracy){
            if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
                turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }else{
                turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
            }
            lF.setPower(power+turnOffset);
            lB.setPower(-power+turnOffset);
            rF.setPower(-power-turnOffset);
            rB.setPower(power-turnOffset);
            if(ln.isStopRequested()){return;}
        }
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }

    public void stopDrive(){
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }

    public void angleStrafe(double power, double angle){
        double correctionIntensity = percentCorrection * power;
        power *= (1-percentCorrection);
        if (getStatus() != Status.ANGLE_STRAFING) {
            startAngle = getHeading();
        }
        this.setStatus(Status.ANGLE_STRAFING);
        double turnOffset;
        if(ACMath.compassAngleShorter(getHeading(), startAngle)) {
            turnOffset = Range.clip(correctionIntensity * (ACMath.toCompassAngle(getHeading()) - ACMath.toCompassAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }else{
            turnOffset = Range.clip(correctionIntensity * (ACMath.toStandardAngle(getHeading()) - ACMath.toStandardAngle(startAngle)) / maxCorrectionAngle, -correctionIntensity, correctionIntensity);
        }
        lF.setPower(power*Math.sin(angle + Math.PI/4)+turnOffset);
        lB.setPower(power*Math.sin(angle - Math.PI/4)+turnOffset);
        rB.setPower(power*Math.sin(angle + Math.PI/4)-turnOffset);
        rF.setPower(power*Math.sin(angle - Math.PI/4)-turnOffset);
    }
    public void goToDeltaPosition(double power, double meters, double targetAngle){
        turn(power, targetAngle);
        forward(power, meters);
    }
    public void goToAbsolutePosition(double power, double x, double y)throws InterruptedException{
        double deltaX = x-odometryX;
        double deltaY = y-odometryY;
        double angle = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaY, deltaX);
        ln.telemetry.addData("targetAngle:", angle);
        ln.telemetry.addData("Distance:", distance);
        ln.telemetry.update();
        Thread.sleep(5000);
        goToDeltaPosition(distance, angle, power);
    }

    //Manips methods
    public void setGrab(double position){
        grabL.setPosition(position);
        grabR.setPosition(position);
    }

    public void setArm(double power, double position){
        while(Math.abs(position-arm.getCurrentPosition()) > armAccuracy){
            arm.setPower(power*(Math.abs(position-arm.getCurrentPosition())/(position-arm.getCurrentPosition())));
            if(ln.isStopRequested())return;
            ln.telemetry.addData("Arm position", arm.getCurrentPosition());
            ln.telemetry.addData("Goal position", position);
            ln.telemetry.update();
        }
        arm.setPower(0);
    }

    //Misc methods
    public double getHeading(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.getPosition();
        return ACMath.toStandardAngle(Math.toRadians(angles.firstAngle)+initialAngle);
    }
    public String toString(){
        String value = "Mark 5\n";
        value += "Status: " + robotStatus;
        value += "leftFront Config: " + lF.toString();
        value += "leftFront encoder: " + lF.getCurrentPosition() + "\n";
        value += "leftBack Config: " + lB.toString();
        value += "leftBack encoder: " + lB.getCurrentPosition() + "\n";
        value += "rightFront Config: " + rF.toString();
        value += "rightFront encoder: " + rF.getCurrentPosition() + "\n";
        value += "rightBack Config: " + rB.toString();
        value += "rightBack encoder: " + rB.getCurrentPosition() + "\n";
        value += "Odometry position: (" + odometryX + ", " + odometryY + ")" + "\n";
        value += "Heading angle: " + getHeading() + "\n";
        value += "Skystone: " + skystone + "\n";
        value += "Robot running in " + ln + "\n";
        return value;
    }
}
