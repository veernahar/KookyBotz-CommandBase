package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class IntakeSubsystem extends SubsystemBase {
    protected MotorEx m_intakeMotor;

    public IntakeSubsystem(MotorEx intakeMotor) {
        m_intakeMotor = intakeMotor;
    }

    public void on() {
        m_intakeMotor.motorEx.setPower(1);
    }

    public void off() {
        m_intakeMotor.motorEx.setPower(0);
    }

    public void reverse() {
        m_intakeMotor.motorEx.setPower(-1);
    }
}
