package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.CloseIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class CloseIntakeAndExtendCommand extends SequentialCommandGroup {

    public CloseIntakeAndExtendCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
                                       LiftSubsystem liftSubsystem, OuttakeSubsystem outtakeSubsystem) {

        addCommands(
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
