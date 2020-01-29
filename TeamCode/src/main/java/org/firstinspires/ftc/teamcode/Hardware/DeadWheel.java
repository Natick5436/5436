package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;

public class DeadWheel{

    private String configName;
    private double ticksPerRev;
    private double wheelCircum;
    private DcMotor device;
    private int direction;



    public DeadWheel(DcMotor configName){
        this.direction = 1;
        device = configName;
    }
    public void setDirection(int d) {
        direction = d;
    }
    public int getCurrentPosition(){
        return direction*device.getCurrentPosition();
    }
}
