package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubsystem extends SubsystemBase {

    private MotorEx leftMotor;
    private MotorEx rightMotor;

    public int min;
    public int max;
    public int current = 0;

    public LiftSubsystem(MotorEx leftMotor, MotorEx rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        min = 0;
        max = 1400;

        rightMotor.motor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setPos(int pos) {
        if (pos <= max && pos >= min) current = pos;
        System.out.println(current);
        normalize();
    }

    public void normalize() {
        leftMotor.motorEx.setTargetPosition(current);
        leftMotor.motorEx.setTargetPositionTolerance(10);
        leftMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftMotor.motorEx.setPower(1);
        rightMotor.motorEx.setTargetPosition(current);
        rightMotor.motorEx.setTargetPositionTolerance(10);
        rightMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.motorEx.setPower(1);
    }

    public void outtake() {
        setPos(800);
    }

    public void intake() {
        setPos(0);
    }
}
