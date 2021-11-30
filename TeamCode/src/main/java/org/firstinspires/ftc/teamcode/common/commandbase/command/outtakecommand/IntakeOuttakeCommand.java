package org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class IntakeOuttakeCommand implements Command {
    private OuttakeSubsystem outtake;

    public IntakeOuttakeCommand(OuttakeSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void execute() {
        outtake.intake();
    }
}
