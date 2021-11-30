package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem implements Subsystem {
    private Motor<DcMotorEx> intakeMotor;
    private Servo gateServo;
    private RangeSensor distanceSensor;

    public IntakeSubsystem(Motor<DcMotorEx> intakeMotor, Servo gateServo, RangeSensor distanceSensor) {
        this.intakeMotor = intakeMotor;
        this.gateServo = gateServo;
        this.distanceSensor = distanceSensor;
    }

    public void start() {
        intakeMotor.setSpeed(1);
    }

    public void stop() {
        intakeMotor.setSpeed(0);
    }

    public void reverse() {
        intakeMotor.setSpeed(-1);
    }

    public void open() {
        gateServo.setPosition(0);
    }

    public void close() {
        gateServo.setPosition(1);
    }

    public boolean hasFreight() {
        return distanceSensor.getSensorValue(DistanceUnit.CM) < 7;
    }
}
