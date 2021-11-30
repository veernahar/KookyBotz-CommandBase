package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;


import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class StopIntakeCommand implements Command {
    private IntakeSubsystem intake;

    public StopIntakeCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void execute() {
        intake.stop();
    }
}
