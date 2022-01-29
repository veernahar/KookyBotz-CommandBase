package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TurretSubsystem extends SubsystemBase {
    private final Servo turretServo;

    public static double intakePosition = 0.546;
    public static double outtakeRedPosition = 0.93;
    public static double outtakeBluePosition = 0.162;

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

    public void left() {
        turretServo.setPosition(turretServo.getPosition() + 0.01);
    }

    public void right() {
        turretServo.setPosition(turretServo.getPosition() - 0.01);
    }
}
