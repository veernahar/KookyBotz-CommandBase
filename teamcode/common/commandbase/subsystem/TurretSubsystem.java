package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
    private Servo turretServo;


    public TurretSubsystem(Servo turretServo) {
        this.turretServo = turretServo;

    }

    public void intake() {
        turretServo.setPosition(0.565);

    }

    public void outtakeBlue() {
        turretServo.setPosition(0.565);
    }

    public void outtakeRed() {
        turretServo.setPosition(0.565);
    }
}
