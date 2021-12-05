package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final MotorEx leftMotor;
    private final MotorEx rightMotor;

    public static int min = 0;
    public static int max = 600;

    public static int intakePos = 12;
    public static int outtakePos = 600;

    public int current = 0;

    public LiftSubsystem(MotorEx leftMotor, MotorEx rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

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
        setPos(outtakePos);
    }

    public void intake() {
        setPos(intakePos);
    }
}
