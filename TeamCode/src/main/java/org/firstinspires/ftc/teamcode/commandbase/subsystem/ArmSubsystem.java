package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    protected Servo m_leftArmServo;
    protected Servo m_rightArmServo;

    public ArmSubsystem(Servo leftArmServo, Servo rightArmServo) {
        m_leftArmServo = leftArmServo;
        m_rightArmServo = rightArmServo;
    }

    public void intake() {
        m_leftArmServo.setPosition(0);
        m_rightArmServo.setPosition(1);
    }

    public void outtake() {
        m_leftArmServo.setPosition(0.65);
        m_rightArmServo.setPosition(0.35);
    }
}
