package org.firstinspires.ftc.teamcode.common.commandbase.command.drivecommand;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.rr.AutonomousDrivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class MoveForwardUntilIntakeCommand extends CommandBase {
    private AutonomousDrivetrain drive;
    private IntakeSubsystem intake;
    private double power;

    public MoveForwardUntilIntakeCommand(AutonomousDrivetrain autonomousDrivetrain, IntakeSubsystem intake, double power) {
        drive = autonomousDrivetrain;
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void initialize() {
        drive.setWeightedDrivePower(new Pose2d(power, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return intake.hasFreight();
    }

    @Override
    public void end(boolean interrupted) {
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }
}
