package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
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
