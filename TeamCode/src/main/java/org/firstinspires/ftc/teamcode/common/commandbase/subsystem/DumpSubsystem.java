package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DumpSubsystem extends SubsystemBase {
    private final Servo outtakeServo;

    public static double intakePosition = 0.55;
    public static double restPosition = 0.475;
    public static double outtakePosition = 0;

    public DumpSubsystem(Servo outtakseServo) {
        this.outtakeServo = outtakseServo;
    }

    public void intake() {
        outtakeServo.setPosition(intakePosition);
    }

    public void rest() {
        outtakeServo.setPosition(restPosition);
    }

    public void outtake() {
        outtakeServo.setPosition(outtakePosition);
    }
}
