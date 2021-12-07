package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class Robot {
    public final LiftSubsystem lift;
    public final ArmSubsystem arm;
    public final IntakeSubsystem intake;
    public final DumpSubsystem dump;
    public final TurretSubsystem turret;
    public final DuckSubsystem ducc;

    public Robot(HardwareMap hardwareMap) {


        MotorEx leftSlide = new MotorEx(hardwareMap, "slideL");
        MotorEx rightSlide = new MotorEx(hardwareMap, "slideR");

        lift = new LiftSubsystem(leftSlide, rightSlide);

        Servo leftArm = hardwareMap.get(Servo.class, "arm1");
        Servo rightArm = hardwareMap.get(Servo.class, "arm2");

        arm = new ArmSubsystem(leftArm, rightArm);

        MotorEx intakeMotor = new MotorEx(hardwareMap, "intake");
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        intake = new IntakeSubsystem(intakeMotor, distanceSensor);

        Servo outtakeServo = hardwareMap.get(Servo.class, "dump");

        dump = new DumpSubsystem(outtakeServo);

        Servo turretServo = hardwareMap.get(Servo.class, "turret");

        turret = new TurretSubsystem(turretServo);

        CRServo leftDucc = hardwareMap.get(CRServo.class, "leftducc");
        CRServo rightDucc = hardwareMap.get(CRServo.class, "rightducc");

        ducc = new DuckSubsystem(leftDucc, rightDucc);
    }
}
