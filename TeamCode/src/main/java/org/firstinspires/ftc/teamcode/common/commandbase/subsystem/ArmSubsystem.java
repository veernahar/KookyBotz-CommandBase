package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

@Config
public class ArmSubsystem extends SubsystemBase {
    private final Servo leftArmServo;
    private final Servo rightArmServo;

    public static double leftArmIntakePosition = 0.14-0.10;
    public static double rightArmIntakePosition = 0.78+0.165;

    public static double leftArmRestPosition = 0.3-0.10;
    public static double rightArmRestPosition = 0.62+0.165;

    public static double leftArmOuttakePosition = 0.61-0.10;
    public static double rightArmOuttakePosition = 0.31+0.165;

    public static double leftArmMidPosition = 0.80-0.10;
    public static double rightArmMidPosition = 0.12+0.165;

    public static double leftArmLowPosition = 0.88-0.10;
    public static double rightArmLowPosition = 0.04+0.165;

    public static double leftArmSharedPosition = 0.84-0.10;
    public static double rightArmSharedPosition = 0.07+0.165;


    public ArmSubsystem(Servo leftArmServo, Servo rightArmServo) {
        this.leftArmServo = leftArmServo;
        this.rightArmServo = rightArmServo;

    }

    public void intake() {
        leftArmServo.setPosition(leftArmIntakePosition);
        rightArmServo.setPosition(rightArmIntakePosition);
    }

    public void outtake() {
        leftArmServo.setPosition(leftArmOuttakePosition);
        rightArmServo.setPosition(rightArmOuttakePosition);
    }

    public void mid() {
        leftArmServo.setPosition(leftArmMidPosition);
        rightArmServo.setPosition(rightArmMidPosition);
    }

    public void low() {
        leftArmServo.setPosition(leftArmLowPosition);
        rightArmServo.setPosition(rightArmLowPosition);
    }

    public void shared() {
        leftArmServo.setPosition(leftArmSharedPosition);
        rightArmServo.setPosition(rightArmSharedPosition);
    }

    public void rest(){
        leftArmServo.setPosition(leftArmRestPosition);
        rightArmServo.setPosition(rightArmRestPosition);
    }
}
