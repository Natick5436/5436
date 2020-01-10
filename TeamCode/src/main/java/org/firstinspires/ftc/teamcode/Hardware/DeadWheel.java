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



    public DeadWheel(HardwareMap hardwareMap, String configName, double ticksPerRev, double wheelCircum){
        this.configName = configName;
        this.ticksPerRev = ticksPerRev;
        this.wheelCircum = wheelCircum;
        device = hardwareMap.dcMotor.get(configName);
    }
}
