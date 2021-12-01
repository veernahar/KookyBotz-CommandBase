package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class OuttakeBlueTurretCommand extends CommandBase {
    private TurretSubsystem turret;

    public OuttakeBlueTurretCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void initialize() {
        turret.outtakeBlue();
    }
}
