package org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand;

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

    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private Timer timer;
    private Pose2d target;
    private double maxPower;
    private Pose2d divisor;

    public HoldPositionCommand(MecanumDrive drivetrain, OdometrySubsystem odometrySubsystem, Pose2d targetPose, long millis, Pose2d divisorPose, double maxPowerDouble) {
        target = targetPose;
        drive = drivetrain;
        odometry = odometrySubsystem;
        maxPower = maxPowerDouble;
        divisor = divisorPose;
        timer = new Timer(millis, TimeUnit.MILLISECONDS);
        setName(m_name + ": " + millis + " milliseconds");
    }

    @Override
    public void initialize() {
        timer.start();
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d pose = odometry.getPose();
        double xPower = (target.getX() - pose.getX()) / divisor.getX();
        double yPower = (target.getY() - pose.getY()) / divisor.getY();
        double txPower = (target.getHeading() - pose.getHeading()) / divisor.getHeading();

        if (Math.abs(xPower) > maxPower) xPower = Math.signum(xPower) * maxPower;
        if (Math.abs(yPower) > maxPower) yPower = Math.signum(yPower) * maxPower;
        if (Math.abs(txPower) > maxPower) txPower = Math.signum(txPower) * maxPower;

        drive.driveRobotCentric(yPower, xPower, txPower);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        timer.pause();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }


    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
