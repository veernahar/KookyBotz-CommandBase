package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.subsystem.Subsystem;


public class LiftSubsystem implements Subsystem {
    //TODO missing some features dont recommend abstracting encoded motors unless u doing rr pid
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    public int min;
    public int max;
    public int current = 0;

    public LiftSubsystem(EncodedMotor<DcMotorEx> leftMotor, EncodedMotor<DcMotorEx> rightMotor) {
        this.leftMotor = leftMotor.invert().getDevice();
        this.rightMotor = rightMotor.getDevice();

    }

    public void setPos(int pos) {
        if (pos <= max && pos >= min) current = pos;
        normalize();
    }

    public void normalize() {
        leftMotor.setTargetPosition(current);
        leftMotor.setTargetPositionTolerance(10);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(1);
        rightMotor.setTargetPosition(current);
        rightMotor.setTargetPositionTolerance(10);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(1);
    }

    public void outtake() {
        setPos(1400);
    }

    public void intake() {
        setPos(0);
    }
}
