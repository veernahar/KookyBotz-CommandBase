package org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftRetractCommand extends CommandBase {
    private LiftSubsystem lift;

    public LiftRetractCommand(LiftSubsystem liftSubsystem) {
        lift = liftSubsystem;
    }

    @Override
    public void initialize() {
        lift.intake();
        System.out.println("retract");
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
