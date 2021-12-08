package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DumpSubsystem extends SubsystemBase {
    private final Servo outtakeServo;

    public static double intakePosition = 0.675;
    public static double restPosition = 0.6;
    public static double restPosition2 = 0.85;
    public static double outtakePosition = 0.4;

    public DumpSubsystem(Servo outtakseServo) {
        this.outtakeServo = outtakseServo;
    }

    public void intake() {
        outtakeServo.setPosition(intakePosition);
    }

    public void rest1() {
        outtakeServo.setPosition(restPosition);
    }

    public void rest2() {
        outtakeServo.setPosition(restPosition2);
    }

    public void shared(){

    }

    public void outtake() {
        outtakeServo.setPosition(outtakePosition);
    }
}
