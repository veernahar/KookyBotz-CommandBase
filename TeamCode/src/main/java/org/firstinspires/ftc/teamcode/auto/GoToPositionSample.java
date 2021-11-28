package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.commandbase.command.GoToPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.HoldPositionCommand;

@Autonomous
public class GoToPositionSample extends CommandOpMode {

    // define our constants
    static final double TRACKWIDTH = 9.38101;
    static final double WHEEL_DIAMETER = 1.37795276;
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = -9;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private GoToPositionCommand gtpCommand1, gtpCommand2;
    private HoldPositionCommand hpCommand1, hpCommand2;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fL = new Motor(hardwareMap, "lf");
        fR = new Motor(hardwareMap, "rf");
        bL = new Motor(hardwareMap, "lb");
        bR = new Motor(hardwareMap, "rb");

        fL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        leftEncoder = new MotorEx(hardwareMap, "rb");
        rightEncoder = new MotorEx(hardwareMap, "rf");
        centerEncoder = new MotorEx(hardwareMap, "lf");


        leftEncoder.encoder.setDirection(Motor.Direction.REVERSE);


        // calculate multiplier
        TICKS_TO_INCHES = 1 / 1743.02601133;

        double left = leftEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        double right = rightEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        double center = centerEncoder.getCurrentPosition() * TICKS_TO_INCHES;


        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES - left,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES - right,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES - center,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        m_odometry = new OdometrySubsystem(m_robotOdometry);

        // create our pure pursuit command
        gtpCommand1 = new GoToPositionCommand(
                m_robotDrive, m_odometry,
                new Pose2d(48, 0, new Rotation2d(0)),
                new Pose2d(0.5, 0.5, new Rotation2d(Math.toRadians(5))),
                new Pose2d(8, 8, new Rotation2d(Math.toRadians(35))),
                0.5
        );

        gtpCommand2 = new GoToPositionCommand(
                m_robotDrive, m_odometry,
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(0.5, 0.5, new Rotation2d(Math.toRadians(5))),
                new Pose2d(8, 8, new Rotation2d(Math.toRadians(35))),
                0.5
        );

        hpCommand1 = new HoldPositionCommand(
                m_robotDrive, m_odometry,
                new Pose2d(48, 0, new Rotation2d(0)),
                1500,
                new Pose2d(8, 8, new Rotation2d(Math.toRadians(35))),
                0.5
        );

        hpCommand2 = new HoldPositionCommand(
                m_robotDrive, m_odometry,
                new Pose2d(0, 0, new Rotation2d(0)),
                1500,
                new Pose2d(8, 8, new Rotation2d(Math.toRadians(35))),
                0.5
        );


        // schedule the command
        schedule(
                new SequentialCommandGroup(
                        gtpCommand1, hpCommand1, new WaitCommand(2500), gtpCommand2, hpCommand2
                )
        );
    }

    @Override
    public void run() {
        super.run();
        Pose2d pose = m_robotOdometry.getPose();
        telemetry.addLine(pose.toString());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("red").setFill("red").fillCircle(pose.getX(), pose.getY(), 9);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}