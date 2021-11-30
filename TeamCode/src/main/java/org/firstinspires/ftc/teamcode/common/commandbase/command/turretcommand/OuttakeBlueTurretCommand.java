package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class OuttakeBlueTurretCommand implements Command {
    private TurretSubsystem turret;

    public OuttakeBlueTurretCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void execute() {
        turret.outtakeBlue();
    }
}
