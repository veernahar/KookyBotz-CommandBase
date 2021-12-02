package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem extends SubsystemBase {
    private Servo outtakeServo;

    public OuttakeSubsystem(Servo outtakeServo) {
        this.outtakeServo = outtakeServo;
    }

    public void intake() {
        outtakeServo.setPosition(0.2);
    }

    public void outtake() {
        outtakeServo.setPosition(1);
    }
}
