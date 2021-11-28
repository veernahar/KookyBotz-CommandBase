package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class PurePursuitSample extends CommandOpMode {

    // define our constants
    static final double TRACKWIDTH = 9.38101;
    static final double WHEEL_DIAMETER = 1.37795276;
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = -9;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
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
        ppCommand = new PurePursuitCommand(
                m_robotDrive, m_odometry,
                new StartWaypoint(0, 0),
                new EndWaypoint(
                        400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1
                )
        );

        // schedule the command
        schedule(ppCommand);
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