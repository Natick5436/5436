package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;

public class REVEncoder extends Thread{
    DigitalChannel channelA;
    DigitalChannel channelB;
    boolean stateA;
    boolean stateB;
    int count;

    LinearOpMode ln;
    public REVEncoder(String nameA, String nameB, LinearOpMode linear){
        ln = linear;
        channelA = ln.hardwareMap.digitalChannel.get(nameA);
        channelB = ln.hardwareMap.digitalChannel.get(nameB);
        channelA.setMode(DigitalChannel.Mode.INPUT);
        channelB.setMode(DigitalChannel.Mode.INPUT);
        stateA = channelA.getState();
        stateB = channelB.getState();
        count = 0;
    }

    public void run(){
        while(ln.opModeIsActive()){
            //First attempt at encoder math
            /*if(stateA&&stateB){
                if(channelA.getState() != stateA){
                    count++;
                    stateA = channelA.getState();
                }
                if(channelB.getState() != stateB){
                    count--;
                    stateB = channelB.getState();
                }
            }else if(stateA&&!stateB){
                if(channelA.getState() != stateA){
                    count--;
                    stateA = channelA.getState();
                }
                if(channelB.getState() != stateB){
                    count++;
                    stateB = channelB.getState();
                }
            }else if(!stateA&&stateB){
                if(channelA.getState() != stateA){
                    count--;
                    stateA = channelA.getState();
                }
                if(channelB.getState() != stateB){
                    count++;
                    stateB = channelB.getState();
                }
            }else if(!stateA&&!stateB){
                if(channelA.getState() != stateA){
                    count++;
                    stateA = channelA.getState();
                }
                if(channelB.getState() != stateB){
                    count--;
                    stateB = channelB.getState();
                }
            }*/

            //Simpler attempt at encoder math with less execution lines.
            if(channelA.getState()!=stateA){
                if(stateA==stateB){
                    count++;
                }else{
                    count--;
                }
                stateA = channelA.getState();
            }else if(channelB.getState()!=stateB){
                if(stateA==stateB){
                    count--;
                }else{
                    count++;
                }
                stateB = channelB.getState();
            }
        }
    }
}
