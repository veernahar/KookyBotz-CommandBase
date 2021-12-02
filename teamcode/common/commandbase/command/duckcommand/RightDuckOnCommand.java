package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class RightDuckOnCommand extends CommandBase {
    private DuckSubsystem duck;

    public RightDuckOnCommand(DuckSubsystem duckSubsystem){
        duck = duckSubsystem;
    }

    @Override
    public void initialize(){
        duck.rightOn();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
