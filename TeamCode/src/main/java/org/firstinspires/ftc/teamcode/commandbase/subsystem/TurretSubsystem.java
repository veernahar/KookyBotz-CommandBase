package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
    protected Servo m_turretServo;


    public TurretSubsystem(Servo turretServo) {
        m_turretServo = turretServo;

    }

    public void intake() {
        m_turretServo.setPosition(0.5);

    }

    public void outtakeBlue() {
        m_turretServo.setPosition(0.1);
    }

    public void outtakeRed() {
        m_turretServo.setPosition(0.9);
    }
}
