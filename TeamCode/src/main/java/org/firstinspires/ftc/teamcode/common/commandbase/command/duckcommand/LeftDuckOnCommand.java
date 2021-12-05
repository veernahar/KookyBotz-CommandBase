package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class LeftDuckOnCommand extends CommandBase {
    private DuckSubsystem duck;

    public LeftDuckOnCommand(DuckSubsystem duckSubsystem){
        duck = duckSubsystem;
    }

    @Override
    public void initialize(){
        duck.leftOn();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
