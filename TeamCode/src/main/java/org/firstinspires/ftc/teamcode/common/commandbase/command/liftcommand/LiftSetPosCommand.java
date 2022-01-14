package org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftSetPosCommand extends CommandBase {
    private LiftSubsystem lift;
    private int pos;

    public LiftSetPosCommand(LiftSubsystem liftSubsystem, int pos) {
        lift = liftSubsystem;
        this.pos = pos;
    }

    @Override
    public void initialize() {
        lift.setPos(pos);
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
