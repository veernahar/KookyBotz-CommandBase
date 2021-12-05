package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;

public class ArmOuttakeCommand extends CommandBase {
    private ArmSubsystem arm;

    public ArmOuttakeCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
    }

    @Override
    public void initialize() {
        arm.outtake();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
