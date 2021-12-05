package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private IntakeSubsystem intake;

    public IntakeStopCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intake.stop();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
