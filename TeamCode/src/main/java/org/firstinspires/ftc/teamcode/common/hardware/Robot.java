package org.firstinspires.ftc.teamcode.common.hardware;



import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.logger.Loggable;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class Robot implements Loggable {

    //TODO break this class up and add annotation logging

    public final DrivebaseSubsystem drive;
    private final EncodedMotor<DcMotorEx> rf, rb, lf, lb;
    public final LiftSubsystem lift;
    private final EncodedMotor<DcMotorEx> leftSlide, rightSlide;
    public final ArmSubsystem arm;
    private final Servo leftArm, rightArm;
    public final IntakeSubsystem intake;
    private final Motor<DcMotorEx> intakeMotor;
    private final Servo gate;
    private final RangeSensor distanceSensor;
    public final OuttakeSubsystem outtake;
    private final Servo outtakeServo;
    public final TurretSubsystem turret;
    private final Servo turretServo;
    public final DuckSubsystem ducc;
    private final Motor<CRServo> leftDucc, rightDucc;

    public Robot() {
        rf = new EncodedMotor<>("rf");
        rb = new EncodedMotor<>("rb");
        lf = new EncodedMotor<>("lf");
        lb = new EncodedMotor<>("lb");

        //TODO add in ids for imu and odometry subsystem and its encoders
        // new OdometrySubsystem
        // new IMU
        // new MotorEncoders

        drive = new DrivebaseSubsystem(lf, rf, lb, rb, null, null);


        leftSlide = new EncodedMotor<>("leftSlide");
        rightSlide = new EncodedMotor<>("rightSlide");

        lift = new LiftSubsystem(leftSlide, rightSlide);

        leftArm = new Servo("leftArm");
        rightArm = new Servo("rightArm");

        arm = new ArmSubsystem(leftArm, rightArm);

        intakeMotor = new Motor<>("intake");
        gate = new Servo("gate");
        distanceSensor = new RangeSensor("distance");

        intake = new IntakeSubsystem(intakeMotor, gate, distanceSensor);

        outtakeServo = new Servo("outtake");

        outtake = new OuttakeSubsystem(outtakeServo);

        turretServo = new Servo("turret");

        turret = new TurretSubsystem(turretServo);

        leftDucc = new Motor<>("leftDucc");
        rightDucc = new Motor<>("rigthDucc");

        ducc = new DuckSubsystem(leftDucc, rightDucc);
    }
}
