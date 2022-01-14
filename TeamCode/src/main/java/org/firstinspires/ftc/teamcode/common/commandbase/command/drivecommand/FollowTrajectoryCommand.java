package org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.AutonomousDrivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class FollowTrajectoryCommand extends CommandBase {
    private AutonomousDrivetrain drive;
    private TrajectorySequence traj;

    public FollowTrajectoryCommand(AutonomousDrivetrain rrMecanumDrive, TrajectorySequence trajectorySequence) {
        drive = rrMecanumDrive;
        traj = trajectorySequence;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(traj);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
