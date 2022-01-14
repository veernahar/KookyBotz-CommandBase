package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;

public class ArmIntakeCommand extends CommandBase {
    private ArmSubsystem arm;

    public ArmIntakeCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
    }

    @Override
    public void initialize() {
        arm.intake();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
