package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class DuckOffCommand extends CommandBase {
    private DuckSubsystem duck;

    public DuckOffCommand(DuckSubsystem duckSubsystem) {
        duck = duckSubsystem;
    }

    @Override
    public void initialize() {
        duck.off();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
