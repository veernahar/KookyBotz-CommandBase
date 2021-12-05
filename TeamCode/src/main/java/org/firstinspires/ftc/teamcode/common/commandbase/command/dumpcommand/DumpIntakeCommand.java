package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;

public class DumpIntakeCommand extends CommandBase {
    private DumpSubsystem outtake;

    public DumpIntakeCommand(DumpSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.intake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
