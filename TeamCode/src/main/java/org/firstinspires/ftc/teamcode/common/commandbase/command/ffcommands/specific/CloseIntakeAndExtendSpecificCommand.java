package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands.specific;

import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.CloseIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class CloseIntakeAndExtendSpecificCommand extends SequentialCommandGroup {

    public CloseIntakeAndExtendSpecificCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
                                               LiftSubsystem liftSubsystem, OuttakeSubsystem outtakeSubsystem) {

        super(
                new WaitCommand(250),
                new CloseIntakeCommand(intakeSubsystem),
                new WaitCommand(500),
                new ReverseIntakeCommand(intakeSubsystem),
                new OuttakeArmCommand(armSubsystem),
                new WaitCommand(500),
                new LiftExtendCommand(liftSubsystem)
        );

        addRequirements(intakeSubsystem, armSubsystem, liftSubsystem, outtakeSubsystem);
    }
}
