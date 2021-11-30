package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;

public class ArmSubsystem implements Subsystem {
    //TODO can make this a servo group
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
