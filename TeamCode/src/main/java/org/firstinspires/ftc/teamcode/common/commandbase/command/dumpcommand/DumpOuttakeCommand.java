package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;

public class DumpOuttakeCommand extends CommandBase {
    private DumpSubsystem outtake;

    public DumpOuttakeCommand(DumpSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.outtake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
