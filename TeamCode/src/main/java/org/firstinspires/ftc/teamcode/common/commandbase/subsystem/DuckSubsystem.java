package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class DuckSubsystem extends SubsystemBase {
    DcMotorEx motor;
    public static double slowPower = 0.75;

    public DuckSubsystem(DcMotorEx duck) {
        this.motor = duck;
    }

    public void red() {
        motor.setPower(slowPower);
    }


    public void blue() {
        motor.setPower(-slowPower);
    }

    public void off() {
        motor.setPower(0);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

}
