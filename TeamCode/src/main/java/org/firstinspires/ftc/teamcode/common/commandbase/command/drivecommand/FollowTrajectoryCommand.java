package org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class FollowTrajectoryCommand extends CommandBase {
    private RRMecanumDrive drive;
    private TrajectorySequence traj;

    public FollowTrajectoryCommand(RRMecanumDrive rrMecanumDrive, TrajectorySequence trajectorySequence) {
        drive = rrMecanumDrive;
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
}
