package org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;

public class GoToPositionCommand extends CommandBase {

    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private Pose2d target;
    private Pose2d error;
    private double maxPower;
    private Pose2d divisor;

    public GoToPositionCommand(MecanumDrive drivetrain, OdometrySubsystem odometrySubsystem, Pose2d targetPose, Pose2d errorPose, Pose2d divisorPose, double maxPowerDouble) {
        target = targetPose;
        drive = drivetrain;
        odometry = odometrySubsystem;
        error = errorPose;
        maxPower = maxPowerDouble;
        divisor = divisorPose;
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
    }

    @Override
    public boolean isFinished() {
        Pose2d robotPose = odometry.getPose();
        Pose2d robotError = subtractPose(target, robotPose);
        return
                Math.abs(robotError.getX()) < error.getX() &&
                        Math.abs(robotError.getY()) < error.getY() &&
                        Math.abs(robotError.getHeading()) < error.getHeading();
    }

    private Pose2d subtractPose(Pose2d a, Pose2d b) {
        return new Pose2d(
                a.getTranslation().minus(b.getTranslation()),
                a.getRotation().minus(b.getRotation())
        );
    }

}
