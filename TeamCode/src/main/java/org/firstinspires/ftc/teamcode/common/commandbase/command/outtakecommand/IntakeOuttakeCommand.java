package org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class IntakeOuttakeCommand extends CommandBase {
    private OuttakeSubsystem outtake;

    public IntakeOuttakeCommand(OuttakeSubsystem outtakeSubsystem) {
        outtake = outtakeSubsystem;
    }

    @Override
    public void initialize() {
        outtake.intake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
