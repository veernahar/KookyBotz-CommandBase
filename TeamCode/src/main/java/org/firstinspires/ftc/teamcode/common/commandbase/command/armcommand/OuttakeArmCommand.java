package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class OuttakeArmCommand extends CommandBase {
    private ArmSubsystem arm;

    public OuttakeArmCommand(ArmSubsystem armSubsystem) {
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
