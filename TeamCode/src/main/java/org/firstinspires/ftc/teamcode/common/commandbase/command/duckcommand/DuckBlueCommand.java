package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class DuckBlueCommand extends CommandBase {
    private DuckSubsystem duck;

    public DuckBlueCommand(DuckSubsystem duckSubsystem) {
        duck = duckSubsystem;
    }

    @Override
    public void initialize() {
        duck.blue();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
