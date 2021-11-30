package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;

public class TurretSubsystem implements Subsystem {
    private Servo turretServo;


    public TurretSubsystem(Servo turretServo) {
        this.turretServo = turretServo;

    }

    public void intake() {
        turretServo.setPosition(0.5);

    }

    public void outtakeBlue() {
        turretServo.setPosition(0.1);
    }

    public void outtakeRed() {
        turretServo.setPosition(0.9);
    }
}
