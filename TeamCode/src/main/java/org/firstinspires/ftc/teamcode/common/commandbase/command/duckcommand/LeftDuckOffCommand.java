package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class LeftDuckOffCommand implements Command {
    private DuckSubsystem duck;

    public LeftDuckOffCommand(DuckSubsystem duckSubsystem){
        duck = duckSubsystem;
    }

    @Override
    public void execute(){
        duck.leftOff();
    }
}
