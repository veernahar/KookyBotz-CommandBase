package org.firstinspires.ftc.teamcode.common.commandbase.command.dumpcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class OuttakeIntakeCommand extends CommandBase {
    private OuttakeSubsystem outtake;

    public OuttakeIntakeCommand(OuttakeSubsystem outtakeSubsystem) {
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
