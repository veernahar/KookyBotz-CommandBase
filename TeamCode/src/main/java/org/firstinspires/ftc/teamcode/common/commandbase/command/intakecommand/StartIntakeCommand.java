package org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;

public class StartIntakeCommand extends CommandBase {
    private IntakeSubsystem intake;

    public StartIntakeCommand(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intake.start();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
