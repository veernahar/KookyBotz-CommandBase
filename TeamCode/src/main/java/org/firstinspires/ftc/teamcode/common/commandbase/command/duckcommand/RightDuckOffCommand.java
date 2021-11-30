package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;


import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class RightDuckOffCommand implements Command {
    private DuckSubsystem duck;

    public RightDuckOffCommand(DuckSubsystem duckSubsystem) {
        duck = duckSubsystem;
    }

    @Override
    public void execute() {
        duck.rightOff();
    }
}
