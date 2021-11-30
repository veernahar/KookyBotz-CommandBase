package org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class OuttakeArmCommand implements Command {
    private ArmSubsystem arm;

    public OuttakeArmCommand(ArmSubsystem armSubsystem) {
        arm = armSubsystem;
    }

    @Override
    public void execute() {
        arm.outtake();
    }


}
