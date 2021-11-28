package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubsystem extends SubsystemBase {

    protected MotorEx m_leftMotor;
    protected MotorEx m_rightMotor;

    public int min;
    public int max;
    public int current = 0;

    public LiftSubsystem(MotorEx leftMotor, MotorEx rightMotor) {
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;

        m_leftMotor.motor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setPos(int pos) {
        if (pos <= max && pos >= min) current = pos;
        normalize();
    }

    public void normalize() {
        m_leftMotor.motorEx.setTargetPosition(current);
        m_leftMotor.motorEx.setTargetPositionTolerance(10);
        m_leftMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m_leftMotor.motorEx.setPower(1);
        m_rightMotor.motorEx.setTargetPosition(current);
        m_rightMotor.motorEx.setTargetPositionTolerance(10);
        m_rightMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m_rightMotor.motorEx.setPower(1);
    }
}
