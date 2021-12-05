package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    private final Servo outtakeServo;

    public static double intakePosition = 0;
    public static double outtakePosition = 0.65;

    public OuttakeSubsystem(Servo outtakseServo) {
        this.outtakeServo = outtakseServo;
    }

    public void intake() {
        outtakeServo.setPosition(intakePosition);
    }

    public void outtake() {
        outtakeServo.setPosition(outtakePosition);
    }
}
