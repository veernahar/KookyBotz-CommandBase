package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class IntakeTurretCommand implements Command {
    private TurretSubsystem turret;

    public IntakeTurretCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void execute() {
        turret.intake();
    }
}
