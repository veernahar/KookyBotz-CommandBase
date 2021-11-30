package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;


import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class OuttakeRedTurretCommand implements Command {
    private TurretSubsystem turret;

    public OuttakeRedTurretCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void execute() {
        turret.outtakeRed();
    }
}
