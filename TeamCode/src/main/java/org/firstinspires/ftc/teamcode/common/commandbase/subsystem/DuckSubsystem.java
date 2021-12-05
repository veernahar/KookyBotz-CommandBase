package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class DuckSubsystem extends SubsystemBase {
    private final CRServo left;
    private final CRServo right;

    public DuckSubsystem(CRServo left, CRServo right) {
        this.left = left;
        this.right = right;
    }

    public void leftOn() {
        left.setPower(1);
    }

    public void leftOff() {
        left.setPower(0);
    }

    public void rightOn() {
        right.setPower(-1);
    }

    public void rightOff() {
        right.setPower(0);
    }
}
