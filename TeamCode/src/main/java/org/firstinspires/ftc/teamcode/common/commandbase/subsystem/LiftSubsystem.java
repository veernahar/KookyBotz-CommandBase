package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final MotorEx leftMotor;
    private final MotorEx rightMotor;

    public static int min = 0;
    public static int max = 600;

    public static int intakePos = 0;
    public static int outtakePos = 475;

    public int current = 0;

    public LiftSubsystem(MotorEx leftMotor, MotorEx rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftMotor.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.motor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setPos(int pos) {
        if (pos <= max && pos >= min) current = pos;
        System.out.println(current);
        normalize();
    }

    public void normalize() {
        leftMotor.motorEx.setTargetPosition(current);
        leftMotor.motorEx.setTargetPositionTolerance(25);
        leftMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftMotor.motorEx.setPower(0.65);
        rightMotor.motorEx.setTargetPosition(current);
        rightMotor.motorEx.setTargetPositionTolerance(25);
        rightMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.motorEx.setPower(0.65);
    }

    public void outtake() {
        setPos(outtakePos);
    }

    public void intake() {
        setPos(intakePos);
    }

    public double getCurrentDrawA() {
        return leftMotor.motorEx.getCurrent(CurrentUnit.AMPS) + rightMotor.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void periodic() {
        if (Math.abs(leftMotor.motorEx.getTargetPosition() - leftMotor.motorEx.getCurrentPosition()) < 25 && leftMotor.motorEx.getTargetPosition() == intakePos) {
            leftMotor.motorEx.setPower(0);
            rightMotor.motorEx.setPower(0);
        } else {
            normalize();
        }


        System.out.printf(Locale.ENGLISH, "current: %d, target: %d%n", leftMotor.motorEx.getCurrentPosition(), leftMotor.motorEx.getTargetPosition());
    }
}
