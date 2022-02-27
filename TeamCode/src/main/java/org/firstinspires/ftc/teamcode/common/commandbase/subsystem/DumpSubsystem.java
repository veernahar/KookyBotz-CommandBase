package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DumpSubsystem extends SubsystemBase {
    private final Servo outtakeServo;
    private final Servo gateServo;

    public static double intakePosition = 0.9;
    public static double foldPosition = 0.95;
    public static double outtakePosition = 0.6;
    public static double sharedPosition = 0.8;

    public static double closePosition = 0.2;
    public static double openPosition = 0.5;

    public DumpSubsystem(Servo outtakseServo, Servo gateServo) {
        this.outtakeServo = outtakseServo;
        this.gateServo = gateServo;
    }

    public void intake() {
        outtakeServo.setPosition(intakePosition);
    }

    public void fold(){
        outtakeServo.setPosition(foldPosition);
    }

    public void outtake() {
        outtakeServo.setPosition(outtakePosition);
    }

    public void shared() {
        outtakeServo.setPosition(sharedPosition);
    }

    public void close() {
        gateServo.setPosition(closePosition);
    }

    public void open() {
        gateServo.setPosition(openPosition);
    }
}
