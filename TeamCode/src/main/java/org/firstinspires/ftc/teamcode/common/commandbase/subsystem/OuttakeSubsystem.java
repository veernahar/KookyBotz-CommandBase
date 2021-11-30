package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;

public class OuttakeSubsystem implements Subsystem {
    private Servo outtakeServo;

    public OuttakeSubsystem(Servo outtakseServo) {
        this.outtakeServo = outtakseServo;
    }

    public void intake() {
        outtakeServo.setPosition(0);
    }

    public void outtake() {
        outtakeServo.setPosition(0.65);
    }
}
