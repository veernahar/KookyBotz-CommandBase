package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmSubsystem extends SubsystemBase {
    private final Servo leftArmServo;
    private final Servo rightArmServo;

    public static double leftArmIntakePosition = 0.165;
    public static double rightArmIntakePosition = 1 - leftArmIntakePosition;

    public static double leftArmOuttakePosition = 0.635;
    public static double rightArmOuttakePosition = 1 - leftArmOuttakePosition;


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
}
