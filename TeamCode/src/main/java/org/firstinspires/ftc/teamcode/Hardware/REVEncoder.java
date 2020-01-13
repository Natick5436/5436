package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class REVEncoder extends Thread{
    DigitalChannel channelA;
    DigitalChannel channelB;
    boolean stateA;
    boolean stateB;
    boolean currentA;
    boolean currentB;
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

    public int getPosition(){
        return count;
    }

    public void run(){
        while(ln.opModeIsActive() || !ln.isStarted()){
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
            currentA = channelA.getState();
            currentB = channelB.getState();
            //Simpler attempt at encoder math with less execution lines.
            if(currentA!=stateA){
                if(currentA!=currentB){
                    count++;
                }else{
                    count--;
                }
                stateA = currentA;
            }else if(currentB!=stateB){
                if(currentA!=currentB){
                    count--;
                }else{
                    count++;
                }
                stateB = currentB;
            }
        }
    }
}
