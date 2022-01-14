package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;

public class ArmMidCommand extends CommandBase {
    private ArmSubsystem arm;

    public ArmMidCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
    }

    @Override
    public void initialize() {
        arm.mid();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
