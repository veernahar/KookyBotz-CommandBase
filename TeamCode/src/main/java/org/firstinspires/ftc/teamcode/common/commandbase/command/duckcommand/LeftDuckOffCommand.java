package org.firstinspires.ftc.teamcode.common.commandbase.command.duckcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.DuckSubsystem;

public class LeftDuckOffCommand extends CommandBase {
    private DuckSubsystem duck;

    public LeftDuckOffCommand(DuckSubsystem duckSubsystem){
        duck = duckSubsystem;
    }

    @Override
    public void initialize(){
        duck.leftOff();
    }
}
