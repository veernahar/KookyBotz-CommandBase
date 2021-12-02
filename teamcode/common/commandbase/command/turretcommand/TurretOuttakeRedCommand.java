package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class TurretOuttakeRedCommand extends CommandBase {
    private TurretSubsystem turret;

    public TurretOuttakeRedCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void initialize() {
        turret.outtakeRed();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
