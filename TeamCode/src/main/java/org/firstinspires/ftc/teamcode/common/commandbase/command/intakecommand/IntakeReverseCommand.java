package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
    private IntakeSubsystem intake;

    public IntakeReverseCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intake.reverse();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
