package org.firstinspires.ftc.teamcode.common.commandbase.command.ffcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbase.command.armcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intakecommand.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.liftcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand.IntakeOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.outtakecommand.OuttakeOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.OuttakeSubsystem;

public class OuttakeAndResetCommand extends SequentialCommandGroup {

    public OuttakeAndResetCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
                                  LiftSubsystem liftSubsystem, OuttakeSubsystem outtakeSubsystem) {

        addCommands(
                new OuttakeOuttakeCommand(outtakeSubsystem),
                new WaitCommand(1000),
                new IntakeOuttakeCommand(outtakeSubsystem),
                new LiftRetractCommand(liftSubsystem),
                new WaitCommand(750),
                new IntakeArmCommand(armSubsystem),
                new WaitCommand(500),
                new StartIntakeCommand(intakeSubsystem)
        );

        addRequirements(intakeSubsystem, armSubsystem, liftSubsystem, outtakeSubsystem);
    }
}
