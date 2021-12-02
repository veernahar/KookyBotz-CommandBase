package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class IntakeCloseCommand extends CommandBase {
    private IntakeSubsystem intake;

    public IntakeCloseCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intake.close();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
