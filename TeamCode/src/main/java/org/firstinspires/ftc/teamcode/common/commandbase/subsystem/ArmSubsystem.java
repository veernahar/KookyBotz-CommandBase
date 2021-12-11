package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

@Config
public class ArmSubsystem extends SubsystemBase {
    private final Servo leftArmServo;
    private final Servo rightArmServo;

    public static double leftArmIntakePosition = 0.14;
    public static double rightArmIntakePosition = 0.78;

    public static double leftArmOuttakePosition = 0.61;
    public static double rightArmOuttakePosition = 0.31;

    public static double leftArmSharedPosition = 0.8;
    public static double rightArmSharedPosition = 0.11;


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

    public void shared(){
        leftArmServo.setPosition(leftArmSharedPosition);
        rightArmServo.setPosition(rightArmSharedPosition);
    }

}
