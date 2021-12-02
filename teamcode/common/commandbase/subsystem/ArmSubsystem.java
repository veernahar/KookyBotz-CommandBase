package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo leftArmServo;
    private Servo rightArmServo;

    public ArmSubsystem(Servo leftArmServo, Servo rightArmServo) {
        this.leftArmServo = leftArmServo;
        this.rightArmServo = rightArmServo;
    }

    public void intake() {
        leftArmServo.setPosition(0.97);
        rightArmServo.setPosition(0.03);
    }

    public void outtake() {
        leftArmServo.setPosition(0.3);
        rightArmServo.setPosition(0.7);
    }
}
