package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;

public class DumpSharedCommand extends CommandBase {
    private DumpSubsystem outtake;

    public DumpSharedCommand(DumpSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.shared();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
