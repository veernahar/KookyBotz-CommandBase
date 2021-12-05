package org.firstinspires.ftc.teamcode.common.commandbase.command.turretcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.TurretSubsystem;

public class TurretIntakeCommand extends CommandBase {
    private TurretSubsystem turret;

    public TurretIntakeCommand(TurretSubsystem turretSubsystem) {
        turret = turretSubsystem;
    }

    @Override
    public void initialize() {
        turret.intake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
