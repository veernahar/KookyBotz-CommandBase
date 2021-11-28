package org.firstinspires.ftc.teamcode.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;

/**
 * @author Jackson
 * @see Path
 */
public class GoToPositionCommand extends CommandBase {

    private MecanumDrive m_drive;
    private OdometrySubsystem m_odometry;
    private Pose2d m_target;
    private Pose2d m_error;
    private double m_maxPower;
    private Pose2d m_divisor;

    public GoToPositionCommand(MecanumDrive drive, OdometrySubsystem odometry, Pose2d target, Pose2d error, Pose2d divisor, double maxPower) {
        m_target = target;
        m_drive = drive;
        m_odometry = odometry;
        m_error = error;
        m_maxPower = maxPower;
        m_divisor = divisor;
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
    }

    @Override
    public boolean isFinished() {
        Pose2d robotPose = m_odometry.getPose();
        Pose2d robotError = subtractPose(m_target, robotPose);
        return
                Math.abs(robotError.getX()) < m_error.getX() &&
                        Math.abs(robotError.getY()) < m_error.getY() &&
                        Math.abs(robotError.getHeading()) < m_error.getHeading();
    }

    private Pose2d subtractPose(Pose2d a, Pose2d b) {
        return new Pose2d(
                a.getTranslation().minus(b.getTranslation()),
                a.getRotation().minus(b.getRotation())
        );
    }

}
