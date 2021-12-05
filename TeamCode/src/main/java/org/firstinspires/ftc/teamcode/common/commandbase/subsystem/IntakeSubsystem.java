package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx intakeMotor;
    private DistanceSensor distanceSensor;

    public IntakeSubsystem(MotorEx intakeMotor, DistanceSensor distanceSensor) {
        this.intakeMotor = intakeMotor;
        this.distanceSensor = distanceSensor;
    }

    public void start() {
        intakeMotor.motorEx.setPower(1);
    }

    public void stop() {
        intakeMotor.motorEx.setPower(0);
    }

    public void reverse() {
        intakeMotor.motorEx.setPower(-1);
    }

    public boolean hasFreight() {
        return distanceSensor.getDistance(DistanceUnit.CM) < 7;
    }
}
