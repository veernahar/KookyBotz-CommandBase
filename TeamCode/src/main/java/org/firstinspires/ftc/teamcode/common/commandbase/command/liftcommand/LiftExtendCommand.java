package org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;

public class LiftExtendCommand implements Command {
    private LiftSubsystem lift;

    public LiftExtendCommand(LiftSubsystem liftSubsystem) {
        lift = liftSubsystem;
    }

    @Override
    public void execute() {
        lift.outtake();
    }
}
