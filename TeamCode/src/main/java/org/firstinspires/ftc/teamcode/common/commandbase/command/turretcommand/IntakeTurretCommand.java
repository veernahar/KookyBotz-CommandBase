package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class IntakeTurretCommand extends CommandBase {
    private TurretSubsystem turret;

    public IntakeTurretCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void initialize() {
        turret.intake();
    }
}
