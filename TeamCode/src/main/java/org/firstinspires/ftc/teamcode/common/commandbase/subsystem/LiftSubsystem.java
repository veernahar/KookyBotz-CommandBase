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

    public static int min = -5;
    public static int max = 1550;

    public static int intakePos = -5;
    public static int outtakePos = 1525;

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

    public int getPos() {
        return current;
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

    public void reset() {
        leftMotor.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentDrawA() {
        return leftMotor.motorEx.getCurrent(CurrentUnit.AMPS) + rightMotor.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void periodic(){
        if(!leftMotor.motorEx.isBusy()){
            leftMotor.motorEx.setPower(0);
            rightMotor.motorEx.setPower(0);
        }
    }
}
