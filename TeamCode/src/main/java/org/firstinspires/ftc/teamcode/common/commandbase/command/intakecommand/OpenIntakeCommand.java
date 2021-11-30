package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class OpenIntakeCommand implements Command {
    private IntakeSubsystem intake;

    public OpenIntakeCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void execute() {
        intake.open();
    }
}
