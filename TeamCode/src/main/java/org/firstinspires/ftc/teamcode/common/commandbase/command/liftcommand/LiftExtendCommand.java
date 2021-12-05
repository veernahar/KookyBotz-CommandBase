package org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftExtendCommand extends CommandBase {
    private LiftSubsystem lift;

    public LiftExtendCommand(LiftSubsystem liftSubsystem) {
        lift = liftSubsystem;
    }

    @Override
    public void initialize() {
        lift.outtake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
