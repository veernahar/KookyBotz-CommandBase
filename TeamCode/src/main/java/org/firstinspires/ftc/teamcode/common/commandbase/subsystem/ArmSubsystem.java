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
        leftArmServo.setPosition(0);
        rightArmServo.setPosition(1);
    }

    public void outtake() {
        leftArmServo.setPosition(0.65);
        rightArmServo.setPosition(0.35);
    }
}
