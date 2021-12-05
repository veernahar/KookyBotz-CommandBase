package org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class OuttakeOuttakeCommand extends CommandBase {
    private OuttakeSubsystem outtake;

    public OuttakeOuttakeCommand(OuttakeSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.outtake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
