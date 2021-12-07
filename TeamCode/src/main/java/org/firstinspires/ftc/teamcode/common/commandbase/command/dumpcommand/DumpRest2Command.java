package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DumpSubsystem;

public class DumpRest2Command extends CommandBase {
    private DumpSubsystem outtake;

    public DumpRest2Command(DumpSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.rest2();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
