package org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class OuttakeOuttakeCommand implements Command {
    private OuttakeSubsystem outtake;

    public OuttakeOuttakeCommand(OuttakeSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void execute() {
        outtake.outtake();
    }
}
