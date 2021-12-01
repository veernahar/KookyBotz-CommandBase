package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class CloseIntakeCommand extends CommandBase {
    private IntakeSubsystem intake;

    public CloseIntakeCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intake.close();
    }
}
