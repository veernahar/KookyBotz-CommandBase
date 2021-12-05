package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class TurretOuttakeBlueCommand extends CommandBase {
    private TurretSubsystem turret;

    public TurretOuttakeBlueCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void initialize() {
        turret.outtakeBlue();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
