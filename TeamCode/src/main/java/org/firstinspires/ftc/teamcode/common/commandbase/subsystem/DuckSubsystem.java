package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.Subsystem;

public class DuckSubsystem implements Subsystem {
    private Motor<CRServo> left;
    private Motor<CRServo> right;

    public DuckSubsystem(Motor<CRServo> left, Motor<CRServo> right) {
        this.left = left;
        this.right = right;
    }

    public void leftOn() {
        left.setSpeed(1);
    }

    public void leftOff() {
        left.setSpeed(0);
    }

    public void rightOn() {
        right.setSpeed(-1);
    }

    public void rightOff() {
        right.setSpeed(0);
    }
}
