package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;

public class ArmLowCommand extends CommandBase {
    private ArmSubsystem arm;

    public ArmLowCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
    }

    @Override
    public void initialize() {
        arm.low();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
