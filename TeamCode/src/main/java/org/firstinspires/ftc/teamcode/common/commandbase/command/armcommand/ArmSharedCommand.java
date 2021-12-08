package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;

public class ArmSharedCommand extends CommandBase {
    private ArmSubsystem arm;

    public ArmSharedCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
    }

    @Override
    public void initialize() {
        arm.shared();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
