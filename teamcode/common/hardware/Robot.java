package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class Robot {
    public final MecanumDrive drive;
    private final MotorEx rf, rb, lf, lb;
    public final LiftSubsystem lift;
    private final MotorEx leftSlide, rightSlide;
    public final ArmSubsystem arm;
    private final Servo leftArm, rightArm;
    public final IntakeSubsystem intake;
    private final MotorEx intakeMotor;
    private final Servo gate;
    public final Rev2mDistanceSensor distanceSensor;
    public final OuttakeSubsystem outtake;
    private final Servo outtakeServo;
    public final TurretSubsystem turret;
    private final Servo turretServo;
    public final DuckSubsystem ducc;
    private final CRServo leftDucc, rightDucc;

    public Robot(HardwareMap hardwareMap) {
        rf = new MotorEx(hardwareMap, "rf");
        rb = new MotorEx(hardwareMap, "rb");
        lf = new MotorEx(hardwareMap, "lf");
        lb = new MotorEx(hardwareMap, "lb");

        drive = new MecanumDrive(lf, rf, lb, rb);
        drive.setRightSideInverted(false);

        leftSlide = new MotorEx(hardwareMap, "leftSlide");
        rightSlide = new MotorEx(hardwareMap, "rightSlide");

        lift = new LiftSubsystem(leftSlide, rightSlide);

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        arm = new ArmSubsystem(leftArm, rightArm);

        intakeMotor = new MotorEx(hardwareMap, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        intake = new IntakeSubsystem(intakeMotor, gate, distanceSensor);

        outtakeServo = hardwareMap.get(Servo.class, "outtake");

        outtake = new OuttakeSubsystem(outtakeServo);

        turretServo = hardwareMap.get(Servo.class, "turret");

        turret = new TurretSubsystem(turretServo);

        leftDucc = hardwareMap.get(CRServo.class, "leftDucc");
        rightDucc = hardwareMap.get(CRServo.class, "rightDucc");

        ducc = new DuckSubsystem(leftDucc, rightDucc);
    }
}
