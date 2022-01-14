package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DuckSubsystem extends SubsystemBase {
    DcMotorEx motor;

    public DuckSubsystem(DcMotorEx duck) {
        this.motor = duck;
    }

    public void red() {
        motor.setPower(0.45);
    }


    public void blue() {
        motor.setPower(-0.45);
    }

    public void off() {
        motor.setPower(0);
    }

}
