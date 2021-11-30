package org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftRetractCommand implements Command {
    private LiftSubsystem lift;

    public LiftRetractCommand(LiftSubsystem liftSubsystem) {
        lift = liftSubsystem;
    }

    @Override
    public void execute() {
        lift.intake();
    }
}
