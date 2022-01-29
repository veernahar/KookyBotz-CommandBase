package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;
    private final DistanceSensor distanceSensor;

    public static double intakePower = 1;

    public IntakeSubsystem(MotorEx intakeMotor, DistanceSensor distanceSensor) {
        this.intakeMotor = intakeMotor;
        this.distanceSensor = distanceSensor;
    }

    public void start() {
        intakeMotor.motorEx.setPower(intakePower);
    }

    public void stop() {
        intakeMotor.motorEx.setPower(0);
    }

    public void toggle(){
        if(intakeMotor.motorEx.getPower() == intakePower){
            stop();
        }else{
            start();
        }
    }

    public void reverse() {
        intakeMotor.motorEx.setPower(intakePower);
    }

    public boolean hasFreight() {
        return distanceSensor.getDistance(DistanceUnit.CM) < 5.1;
    }

    public double getCurrentDrawA(){
        return intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS);
    }
}
