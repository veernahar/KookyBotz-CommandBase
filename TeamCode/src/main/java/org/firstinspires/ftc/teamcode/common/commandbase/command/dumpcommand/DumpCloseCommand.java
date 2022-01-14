package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;

public class DumpCloseCommand extends CommandBase {
    private DumpSubsystem outtake;

    public DumpCloseCommand(DumpSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.close();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
