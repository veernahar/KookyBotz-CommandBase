package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;

public class DumpRestCommand extends CommandBase {
    private DumpSubsystem outtake;

    public DumpRestCommand(DumpSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.rest();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
