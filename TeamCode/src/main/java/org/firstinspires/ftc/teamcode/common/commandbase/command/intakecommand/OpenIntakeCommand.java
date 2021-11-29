package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class OpenIntakeCommand extends CommandBase {
    private IntakeSubsystem intake;

    public OpenIntakeCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intake.open();
    }
}
