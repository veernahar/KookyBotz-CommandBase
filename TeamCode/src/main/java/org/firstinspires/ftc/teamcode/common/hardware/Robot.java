package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot {
    public final LiftSubsystem lift;
    public final ArmSubsystem arm;
    public final IntakeSubsystem intake;
    public final DumpSubsystem dump;
    public final TurretSubsystem turret;
    public final DuckSubsystem ducc;
    public final OpenCvWebcam webcam;


    public Robot(HardwareMap hardwareMap) {


        MotorEx leftSlide = new MotorEx(hardwareMap, "leftSlide");
        MotorEx rightSlide = new MotorEx(hardwareMap, "rightSlide");

        leftSlide.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift = new LiftSubsystem(leftSlide, rightSlide);

        Servo leftArm = hardwareMap.get(Servo.class, "arm1");
        Servo rightArm = hardwareMap.get(Servo.class, "arm2");

        arm = new ArmSubsystem(leftArm, rightArm);

        MotorEx intakeMotor = new MotorEx(hardwareMap, "intake");
        intakeMotor.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        intake = new IntakeSubsystem(intakeMotor, distanceSensor);

        Servo outtakeServo = hardwareMap.get(Servo.class, "dump");
        Servo gateServo = hardwareMap.get(Servo.class, "gate");

        dump = new DumpSubsystem(outtakeServo, gateServo);

        Servo turretServo = hardwareMap.get(Servo.class, "turret");

        turret = new TurretSubsystem(turretServo);

        DcMotorEx duckMotor = hardwareMap.get(DcMotorEx.class, "duck");
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ducc = new DuckSubsystem(duckMotor);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    }
}
