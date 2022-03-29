package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CapSubsystem extends SubsystemBase {
    private final Servo cap;

    public static double pickPos = 0.88;
    public static double upPos = 0.55;
    public static double dropPos = 0.63;
    public static double restPos = 0.4;


    public CapSubsystem(Servo cap) {
        this.cap = cap;
    }

    public void pick(){
        cap.setPosition(pickPos);
    }

    public void up(){
        if(cap.getPosition() > upPos) cap.setPosition(cap.getPosition()-0.02);
    }

    public void drop(){
        if(cap.getPosition() < dropPos) cap.setPosition(cap.getPosition()+0.02);
    }

    public void rest(){
        cap.setPosition(restPos);
    }


}
