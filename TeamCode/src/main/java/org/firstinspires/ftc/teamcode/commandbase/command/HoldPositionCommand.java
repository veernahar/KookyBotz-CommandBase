package org.firstinspires.ftc.teamcode.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.util.Timing.Timer;

import java.util.concurrent.TimeUnit;

/**
 * @author Jackson
 * @see Path
 */
public class HoldPositionCommand extends CommandBase {

    private MecanumDrive m_drive;
    private OdometrySubsystem m_odometry;
    protected Timer m_timer;
    private Pose2d m_target;
    private double m_maxPower;
    private Pose2d m_divisor;

    public HoldPositionCommand(MecanumDrive drive, OdometrySubsystem odometry, Pose2d target, long millis, Pose2d divisor, double maxPower) {
        m_target = target;
        m_drive = drive;
        m_odometry = odometry;
        m_maxPower = maxPower;
        m_divisor = divisor;
        m_timer = new Timer(millis, TimeUnit.MILLISECONDS);
        setName(m_name + ": " + millis + " milliseconds");

    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d pose = m_odometry.getPose();
        double xPower = (m_target.getX() - pose.getX()) / m_divisor.getX();
        double yPower = (m_target.getY() - pose.getY()) / m_divisor.getY();
        double txPower = (m_target.getHeading() - pose.getHeading()) / m_divisor.getHeading();

        if (Math.abs(xPower) > m_maxPower) xPower = Math.signum(xPower) * m_maxPower;
        if (Math.abs(yPower) > m_maxPower) yPower = Math.signum(yPower) * m_maxPower;
        if (Math.abs(txPower) > m_maxPower) txPower = Math.signum(txPower) * m_maxPower;

        m_drive.driveRobotCentric(yPower, xPower, txPower);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_timer.pause();
    }

    @Override
    public boolean isFinished() {
        return m_timer.done();
    }

    private Pose2d subtractPose(Pose2d a, Pose2d b) {
        return new Pose2d(
                a.getTranslation().minus(b.getTranslation()),
                a.getRotation().minus(b.getRotation())
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
