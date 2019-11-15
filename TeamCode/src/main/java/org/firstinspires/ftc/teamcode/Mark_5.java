package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//Test comment.
public class Mark_5 {
    DcMotor arm, lF, lB, rF, rB;
    Servo flip, clamp;

    LinearOpMode ln;
    public Mark_5(LinearOpMode linear){
        ln = linear;
    }
    public void initialize(HardwareMap hardwareMap){
        lF = hardwareMap.dcMotor.get("lF");
        lB = hardwareMap.dcMotor.get("lB");
        rF = hardwareMap.dcMotor.get("rF");
        rB = hardwareMap.dcMotor.get("rB");

        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);

        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.dcMotor.get("arm");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flip = hardwareMap.servo.get("flip");
        clamp = hardwareMap.servo.get("clamp");

        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
