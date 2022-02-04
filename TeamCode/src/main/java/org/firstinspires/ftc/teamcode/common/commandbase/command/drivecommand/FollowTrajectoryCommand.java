package org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.AutonomousDrivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class FollowTrajectoryCommand extends CommandBase {
    private AutonomousDrivetrain drive;
    private TrajectorySequence traj;

    public FollowTrajectoryCommand(AutonomousDrivetrain autonomousDrivetrain, TrajectorySequence trajectorySequence) {
        drive = autonomousDrivetrain;
        traj = trajectorySequence;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(traj);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void end(boolean cancel) {
        if (cancel) {
            drive.followTrajectorySequenceAsync(null);
            drive.setDriveSignal(new DriveSignal());
        }
    }
}


