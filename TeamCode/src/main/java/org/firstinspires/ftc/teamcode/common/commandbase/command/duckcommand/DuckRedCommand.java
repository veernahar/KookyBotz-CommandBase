package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class DuckRedCommand extends CommandBase {
    private DuckSubsystem duck;

    public DuckRedCommand(DuckSubsystem duckSubsystem) {
        duck = duckSubsystem;
    }

    @Override
    public void initialize() {
        duck.red();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
