package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TurretSubsystem extends SubsystemBase {
    private final Servo turretServo;

    public static double intakePosition = 0.48;
    public static double outtakeRedPosition = 0.8;
    public static double outtakeBluePosition = 0;

    public TurretSubsystem(Servo turretServo) {
        this.turretServo = turretServo;

    }

    public void intake() {
        turretServo.setPosition(intakePosition);

    }

    public void outtakeBlue() {
        turretServo.setPosition(outtakeBluePosition);
    }

    public void outtakeRed() {
        turretServo.setPosition(outtakeRedPosition);
    }
}
