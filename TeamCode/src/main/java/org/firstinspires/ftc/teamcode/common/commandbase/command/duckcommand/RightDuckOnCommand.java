package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class RightDuckOnCommand implements Command {
    private DuckSubsystem duck;

    public RightDuckOnCommand(DuckSubsystem duckSubsystem){
        duck = duckSubsystem;
    }

    @Override
    public void execute(){
        duck.rightOn();
    }
}
