package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class RightDuckOffCommand extends CommandBase {
    private DuckSubsystem duck;

    public RightDuckOffCommand(DuckSubsystem duckSubsystem) {
        duck = duckSubsystem;
    }

    @Override
    public void initialize() {
        duck.rightOff();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
